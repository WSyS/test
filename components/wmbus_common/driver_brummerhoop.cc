#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"
#include "aes.h"


namespace
{

struct Driver : public virtual MeterCommonImplementation
{
    Driver(MeterInfo &mi, DriverInfo &di);

    bool handleTelegram(AboutTelegram &about, std::vector<uchar> input_frame,
                         bool simulated, std::vector<Address> *addresses,
                         bool *id_match, Telegram *out_analyzed = NULL) override;

    void processContent(Telegram *t) override;
private:
    std::vector<uchar> last_frame_;
};

static bool ok = registerDriver([](DriverInfo &di)
{
    di.setName("brummerhoop");

    di.setMeterType(MeterType::WaterMeter);
    di.setDefaultFields("name,id,total,total_backwards_at_set_date_m3,status,timestamp");

    di.addLinkMode(LinkMode::T1);
    di.addLinkMode(LinkMode::C1);

    di.usesProcessContent();

    di.addDetection(MANUFACTURER_EFE, 0x07, -1);

    di.addDetection(MANUFACTURER_DWZ, 0x07, 0x00); // warm water
    di.addDetection(MANUFACTURER_DWZ, 0x07, 0x02); // alternative

    di.setConstructor([](MeterInfo &mi, DriverInfo &di)
    {
        return std::shared_ptr<Meter>(new Driver(mi, di));
    });
});

static bool hexToBytesFixed16(const std::string &hex,
                               std::array<uint8_t, 16> &out)
{
    // Expect 32 hex chars => 16 bytes AES key
    if (hex.size() != 32)
        return false;

    for (size_t i = 0; i < 16; i++)
    {
        out[i] = (uint8_t)strtoul(hex.substr(i * 2, 2).c_str(), nullptr, 16);
    }
    return true;
}

bool Driver::handleTelegram(AboutTelegram &about, std::vector<uchar> input_frame,
                           bool simulated, std::vector<Address> *addresses,
                           bool *id_match, Telegram *out_analyzed)
{
    last_frame_ = input_frame;

    ESP_LOGI("APP", "(brummerhoop) handleTelegram entered simulated=%d frame_size=%d id_match_ptr=%p addresses_ptr=%p",
             (int)simulated, (int)input_frame.size(), (void *)id_match, (void *)addresses);

    bool parent_ok = MeterCommonImplementation::handleTelegram(about, input_frame,
                                                                 simulated, addresses,
                                                                 id_match, out_analyzed);

    if (out_analyzed != NULL && !out_analyzed->discard)
        processContent(out_analyzed);

    return parent_ok;
}

Driver::Driver(MeterInfo &mi, DriverInfo &di)
    : MeterCommonImplementation(mi, di)
{
    addStringFieldWithExtractorAndLookup(
        "status",
        "Status and error flags.",
        DEFAULT_PRINT_PROPERTIES | PrintProperty::INCLUDE_TPL_STATUS | PrintProperty::STATUS,
        FieldMatcher::build().set(VIFRange::ErrorFlags),
        {
            {
                {
                    "ERROR_FLAGS",
                    Translate::MapType::BitToString,
                    AlwaysTrigger,
                    MaskBits(0xffff),
                    "OK",
                    {
                        {0x01, "SW_ERROR"},
                        {0x02, "CRC_ERROR"},
                        {0x04, "SENSOR_ERROR"},
                        {0x08, "MEASUREMENT_ERROR"},
                        {0x10, "BATTERY_VOLTAGE_ERROR"},
                        {0x20, "MANIPULATION"},
                        {0x40, "LEAKAGE_OR_NO_USAGE"},
                        {0x80, "REVERSE_FLOW"},
                        {0x100, "OVERLOAD"},
                    }
                },
            },
        });

    addNumericFieldWithExtractor(
        "total",
        "The total water consumption recorded by this meter.",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::Volume,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume));

    addNumericFieldWithExtractor(
        "total_backwards",
        "The total backward water volume recorded by this meter.",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::Volume,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyVolumeVIF)
            .add(VIFCombinable::BackwardFlow));
}

void Driver::processContent(Telegram *t)
{
    if (t == NULL)
        return;

    // If the generic parser extracted dv_entries successfully, use the normal
    // pipeline.
    if (!t->dv_entries.empty())
    {
        this->processFieldExtractors(t);
        return;
    }

    // Fallback: brute-force AES-CBC decode of the encrypted user data.
    // Only activate when dv_entries are empty (as requested).
    ESP_LOGW("APP", "(brummerhoop) fallback activated: dv_entries empty");

    if (last_frame_.size() < 32)
        return;

    // Key derivation:
    // YAML uses `key: !secret ...` => MeterInfo::key.
    // meters.cc loads it into meter_keys_.confidentiality_key (16 bytes).
    // We reuse that decoded key here.
    //
    // meter_keys_.confidentiality_key is stored as binary bytes already.
    // If key was not configured, fallback stays inactive.

    auto key_bytes = meterKeys();
    if (key_bytes == NULL)
    {
        ESP_LOGE("APP", "(brummerhoop) AES fallback: meterKeys() is NULL");
        return;
    }

    // meterKeys() returns MeterKeys*; confidentiality_key is 16 bytes.
    // Copy into our fixed array.
    std::array<uint8_t, 16> key_{};
    memcpy(key_.data(), key_bytes->confidentiality_key, 16);

    // Log as hex for debugging.
    char key_hex[33];
    for (size_t i = 0; i < 16; i++)
        sprintf(&key_hex[i * 2], "%02X", key_.data()[i]);
    key_hex[32] = '\0';
    ESP_LOGI("APP", "(brummerhoop) AES key loaded (len=32) %s", key_hex);


    // For Waterstarm/EN13757-3 AES-CBC: the IV is carried in the telegram payload
    // (tpl-cfg shows AES_CBC_IV). In practice, for our received trimmed frame
    // we use the first 16 bytes of last_frame_ as IV.
    // Ciphertext: everything after the first 16 bytes.
    if (last_frame_.size() <= 16)
        return;

    std::array<uint8_t, 16> iv_{};
    memcpy(iv_.data(), last_frame_.data(), 16);

    std::vector<uint8_t> ciphertext;
    ciphertext.reserve(last_frame_.size() - 16);
    for (size_t i = 16; i < last_frame_.size(); i++)
        ciphertext.push_back((uint8_t)last_frame_[i]);

    std::vector<uint8_t> decrypted(ciphertext.size());

    // AES-CBC decrypt
    AES_CBC_decrypt_buffer(decrypted.data(), ciphertext.data(), (uint32_t)ciphertext.size(),
                           key_.data(), iv_.data());



    ESP_LOGI("APP", "(brummerhoop) decrypted payload len=%u", (unsigned)decrypted.size());

    // Search decrypted payload for known patterns:
    // total_m3:  DIF=0x04, VIF=0x13, followed by 4 data bytes (little endian),
    //            where raw is in liters and scale is /1000 => m3.
    // total_backwards: DIF=0x44, VIF=0x93, followed by 2 data bytes (little endian)
    //                   scale /1000.
    auto parse_u32_le_at = [&](size_t pos, bool *ok) -> uint32_t {
        if (pos + 4 > decrypted.size()) {
            *ok = false;
            return 0;
        }
        uint32_t raw = (uint32_t(decrypted[pos + 0])) |
                       (uint32_t(decrypted[pos + 1]) << 8) |
                       (uint32_t(decrypted[pos + 2]) << 16) |
                       (uint32_t(decrypted[pos + 3]) << 24);
        *ok = true;
        return raw;
    };

    auto parse_u16_le_at = [&](size_t pos, bool *ok) -> uint16_t {
        if (pos + 2 > decrypted.size()) {
            *ok = false;
            return 0;
        }
        uint16_t raw = uint16_t(decrypted[pos + 0]) |
                       (uint16_t(decrypted[pos + 1]) << 8);
        *ok = true;
        return raw;
    };

    bool have_total = false;
    bool have_total_backwards = false;

    for (size_t i = 0; i + 6 < decrypted.size(); i++)
    {
        if (!have_total && decrypted[i] == 0x04 && decrypted[i + 1] == 0x13)
        {
            bool okv = false;
            uint32_t raw = parse_u32_le_at(i + 2, &okv);
            if (okv)
            {
                double total_m3 = raw / 1000.0;
                setNumericValue("total", Unit::M3, total_m3);
                ESP_LOGI("APP", "(brummerhoop) fallback total: raw=%u => %.6fm3 at pos=%u",
                         (unsigned)raw, total_m3, (unsigned)i);
                have_total = true;
            }
        }

        if (!have_total_backwards && decrypted[i] == 0x44 && decrypted[i + 1] == 0x93)
        {
            bool okv = false;
            uint16_t raw = parse_u16_le_at(i + 2, &okv);
            if (okv)
            {
                double total_back_m3 = raw / 1000.0;
                setNumericValue("total_backwards", Unit::M3, total_back_m3);
                ESP_LOGI("APP", "(brummerhoop) fallback total_backwards: raw=%u => %.6fm3 at pos=%u",
                         (unsigned)raw, total_back_m3, (unsigned)i);
                have_total_backwards = true;
            }
        }

        if (have_total && have_total_backwards)
            break;
    }

    if (!have_total)
        ESP_LOGW("APP", "(brummerhoop) fallback: total not found in decrypted payload");

    if (!have_total_backwards)
        ESP_LOGW("APP", "(brummerhoop) fallback: total_backwards not found in decrypted payload");

    // Nothing else to do in fallback mode.
}

} // namespace

