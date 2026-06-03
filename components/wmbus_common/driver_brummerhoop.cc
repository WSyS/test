#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"
#include "aes.h"

namespace
{

static constexpr size_t MAX_CT = 128;

struct FrameView {
    const uint8_t* dll = nullptr;
    const uint8_t* tpl = nullptr;
    const uint8_t* payload = nullptr;

    size_t dll_len = 0;
    size_t tpl_len = 0;
    size_t payload_len = 0;
};

/**
 * FIXED: works with std::array (no heap vector anymore)
 */
static bool parseFrame(const std::array<uint8_t, MAX_CT>& f,
                        size_t len,
                        FrameView& out)
{
    if (len < 16)
        return false;

    // DLL (simplified but stable for trimmed ESP frames)
    out.dll = f.data();
    out.dll_len = 1 + 2 + 6;

    if (out.dll_len >= len)
        return false;

    size_t pos = out.dll_len;

    // TPL start
    out.tpl = &f[pos];
    out.tpl_len = len - pos;

    // conservative TPL header skip (CI + ACC + STATUS guess)
    constexpr size_t tpl_header = 4;

    if (pos + tpl_header >= len)
        return false;

    pos += tpl_header;

    // IMPORTANT FIX: remove OMS padding 2F2F
    while (pos + 1 < len &&
           f[pos] == 0x2F &&
           f[pos + 1] == 0x2F)
    {
        pos += 2;
    }

    out.payload = &f[pos];
    out.payload_len = len - pos;

    return true;
}

/**
 * IV builder (wmbusmeters-style)
 */
static void buildIV(const uint8_t* dll,
                     uint8_t acc,
                     std::array<uint8_t,16>& iv)
{
    iv[0] = dll[1];
    iv[1] = dll[2];

    for (int i = 0; i < 6; i++)
        iv[2 + i] = dll[3 + i];

    for (int i = 0; i < 8; i++)
        iv[8 + i] = acc;
}

struct Driver : public virtual MeterCommonImplementation
{
    Driver(MeterInfo &mi, DriverInfo &di);

    bool handleTelegram(AboutTelegram &about,
                        std::vector<uchar> input_frame,
                        bool simulated,
                        std::vector<Address> *addresses,
                        bool *id_match,
                        Telegram *out_analyzed = NULL) override;

    void processContent(Telegram *t) override;

private:
    std::array<uint8_t, MAX_CT> last_frame_{};
    size_t last_len_ = 0;
};

/* registration */
static bool ok = registerDriver([](DriverInfo &di)
{
    di.setName("brummerhoop");

    di.setMeterType(MeterType::WaterMeter);
    di.setDefaultFields("name,id,total,total_backwards,status,timestamp");

    di.addLinkMode(LinkMode::T1);
    di.addLinkMode(LinkMode::C1);

    di.usesProcessContent();

    di.addDetection(MANUFACTURER_EFE, 0x07, -1);
    di.addDetection(MANUFACTURER_DWZ, 0x07, 0x00);
    di.addDetection(MANUFACTURER_DWZ, 0x07, 0x02);

    di.setConstructor([](MeterInfo &mi, DriverInfo &di)
    {
        return std::shared_ptr<Meter>(new Driver(mi, di));
    });
});

Driver::Driver(MeterInfo &mi, DriverInfo &di)
    : MeterCommonImplementation(mi, di)
{
    addNumericFieldWithExtractor(
        "total",
        "Total water consumption",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::Volume,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume));

    addNumericFieldWithExtractor(
        "total_backwards",
        "Backward flow",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::Volume,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyVolumeVIF)
            .add(VIFCombinable::BackwardFlow));
}

bool Driver::handleTelegram(AboutTelegram &about,
                            std::vector<uchar> input_frame,
                            bool simulated,
                            std::vector<Address> *addresses,
                            bool *id_match,
                            Telegram *out_analyzed)
{
    ESP_LOGI("APP",
             "(brummerhoop) handleTelegram frame_size=%d simulated=%d",
             (int)input_frame.size(), (int)simulated);

    bool parent_ok = MeterCommonImplementation::handleTelegram(
        about, input_frame, simulated, addresses, id_match, out_analyzed);

    // cache frame safely
    last_len_ = std::min(input_frame.size(), MAX_CT);
    memcpy(last_frame_.data(), input_frame.data(), last_len_);

    return parent_ok;
}

void Driver::processContent(Telegram *t)
{
    if (!t || !t->dv_entries.empty())
    {
        processFieldExtractors(t);
        return;
    }

    ESP_LOGW("APP", "(brummerhoop) AES fallback activated");

    FrameView frame;
    if (!parseFrame(last_frame_, last_len_, frame))
    {
        ESP_LOGE("APP", "(brummerhoop) frame parse failed");
        return;
    }

    // AES key
    MeterKeys *key_bytes = meterKeys();
    if (!key_bytes || key_bytes->confidentiality_key.size() != 16)
    {
        ESP_LOGE("APP", "(brummerhoop) invalid AES key");
        return;
    }

    std::vector<uchar> key_vec(16);
    memcpy(key_vec.data(), key_bytes->confidentiality_key.data(), 16);

    // ACC from TPL (FIXED)
    uint8_t acc = frame.tpl[1];

    std::array<uint8_t,16> iv{};
    buildIV(frame.dll, acc, iv);

    // ciphertext
    size_t ct_len = frame.payload_len;
    ct_len = (ct_len / 16) * 16;

    if (ct_len < 16)
        return;

    uint8_t decrypted[MAX_CT];
    uint8_t ciphertext[MAX_CT];

    memcpy(ciphertext, frame.payload, ct_len);

    AES_CBC_decrypt_buffer(
        decrypted,
        ciphertext,
        (uint32_t)ct_len,
        safeButUnsafeVectorPtr(key_vec),
        iv.data()
    );

    // DEBUG OUTPUT
    std::string hex;
    hex.reserve(ct_len * 2);

    for (size_t i = 0; i < ct_len; i++)
    {
        char b[3];
        sprintf(b, "%02X", decrypted[i]);
        hex += b;
    }

    ESP_LOGE("APP", "(brummerhoop) decrypted=%s", hex.c_str());

    // PARSE VALUES
    for (size_t i = 0; i + 5 < ct_len; i++)
    {
        if (decrypted[i] == 0x04 && decrypted[i + 1] == 0x13)
        {
            uint32_t raw =
                decrypted[i + 2] |
                (decrypted[i + 3] << 8) |
                (decrypted[i + 4] << 16) |
                (decrypted[i + 5] << 24);

            setNumericValue("total", Unit::M3, raw / 1000.0);
        }

        if (decrypted[i] == 0x44 && decrypted[i + 1] == 0x93)
        {
            uint16_t raw =
                decrypted[i + 2] |
                (decrypted[i + 3] << 8);

            setNumericValue("total_backwards", Unit::M3, raw / 1000.0);
        }
    }
}

} // namespace