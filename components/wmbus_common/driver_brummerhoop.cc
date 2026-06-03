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
    // Avoid heap allocations / capacity churn on ESP32.
    // Your trimmed frames are ~66 bytes (78 bytes raw incl. radio framing).
    static constexpr size_t LAST_FRAME_MAX = 100;
    std::array<uchar, LAST_FRAME_MAX> last_frame_bytes_{};
    size_t last_frame_len_{};
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


    ESP_LOGI("APP", "(brummerhoop) handleTelegram entered simulated=%d frame_size=%d id_match_ptr=%p addresses_ptr=%p",
             (int)simulated, (int)input_frame.size(), (void *)id_match, (void *)addresses);

    bool parent_ok = MeterCommonImplementation::handleTelegram(about, input_frame,
                                                                 simulated, addresses,
                                                                 id_match, out_analyzed);

    // Cache trimmed frame for AES fallback decoding.
    // Without this, last_frame_len_/last_frame_bytes_ stay uninitialized
    // and the fallback returns before the AES key/logging happens.
    last_frame_len_ = input_frame.size() < LAST_FRAME_MAX ? input_frame.size() : LAST_FRAME_MAX;
    if (last_frame_len_ > 0) {
        memcpy(last_frame_bytes_.data(), input_frame.data(), last_frame_len_);
    }


    // Log the configured meter_id (from YAML: wmbus_meter.meter_id) if available.
    // The meter_id configured in YAML is fed into MeterInfo::parse(..., aes="id + "," ...)
    // and ends up embedded in the configured address expression id.
    std::vector<AddressExpression> aexps = this->addressExpressions();
    std::string yaml_meter_id = aexps.size() > 0 ? aexps[0].id : "0";
    yaml_meter_id = std::to_string(std::stoul(yaml_meter_id, nullptr, 16));
    ESP_LOGI("APP", "(brummerhoop) configured meter_id(yaml)=%s", yaml_meter_id.c_str());


    std::string packet_meter_id;

    const std::vector<uchar> &frame = input_frame;
    if (frame.size() > 10)
    {
      // Bytes frame[4..7] correspond to meter id bytes (endianness adjusted for addressExpressions matching).
      uchar id0 = frame[4], id1 = frame[5], id2 = frame[6], id3 = frame[7];
      packet_meter_id = tostrprintf("%02X%02X%02X%02X", id3, id2, id1, id0);
    }


    ESP_LOGI("APP", "(brummerhoop) extracted meter_id(packet)=%s", packet_meter_id.c_str());


    if (packet_meter_id == yaml_meter_id)
    {
        ESP_LOGI("APP", "(brummerhoop) *id_match = true");
        if (id_match) {
            *id_match = true;
        }
    }


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
    // Definitive safety: the AES fallback must never dereference t when it is NULL.
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


    ESP_LOGI("APP", "(brummerhoop) AES fallback debug: last_frame_len_=%u", (unsigned)last_frame_len_);
    if (last_frame_len_ < 32)
        return;




    // AES key derivation:
    // Only accept the YAML configured confidentiality_key.
    MeterKeys *key_bytes = meterKeys();
    if (key_bytes == NULL || key_bytes->confidentiality_key.size() != 16)
    {
        ESP_LOGE("APP", "(brummerhoop) AES fallback: meterKeys() missing/invalid 16-byte AES key");
        return;
    }

    std::array<uint8_t, 16> key_{};
    memcpy(key_.data(), key_bytes->confidentiality_key.data(), 16);



    // Log key hex for debugging.
    char key_hex[33];
    for (size_t i = 0; i < 16; i++)
        sprintf(&key_hex[i * 2], "%02X", key_.data()[i]);
    key_hex[32] = '\0';
    ESP_LOGI("APP", "(brummerhoop) AES key loaded (len=32) %s", key_hex);

    // For Waterstarm/EN13757-3 AES-CBC: the IV is carried in the telegram payload
    // (tpl-cfg shows AES_CBC_IV). In practice, for our received trimmed frame
    // we use the first 16 bytes of last_frame_ as IV.
    // Ciphertext: everything after the first 16 bytes.
    if (last_frame_len_ <= 16)
        return;


    std::array<uint8_t, 16> iv_{};
    memcpy(iv_.data(), last_frame_bytes_.data(), 16);


    // IV construction must match decrypt_TPL_AES_CBC_IV() from wmbus_utils.cc.
    // There: IV = M-field (dll_mfct_b[0..1]) | A-field (dll_a[0..5]) | 8x ACC (tpl_acc)
    // Build IV directly from parsed Telegram fields.

    uint8_t iv_acc = (uint8_t)t->tpl_acc;


    iv_[0] = t->dll_mfct_b[0];
    iv_[1] = t->dll_mfct_b[1];

    for (int j = 0; j < 6; ++j)
        iv_[2 + j] = t->dll_a[j];

    for (int j = 0; j < 8; ++j)
        iv_[8 + j] = iv_acc;

    // Locate ciphertext: EN13757-3 has decrypt check bytes 0x2F2F right after
    // IV-cfg and before ciphertext. The encrypted bytes start immediately
    // after the 2f2f check marker.

    size_t check_pos = std::numeric_limits<size_t>::max();
    for (size_t p = 0; p + 1 < last_frame_len_; p++) {
        if (last_frame_bytes_[p] == 0x2F && last_frame_bytes_[p + 1] == 0x2F) {
            check_pos = p;
            break;
        }
    }

    size_t ct_start = (check_pos != std::numeric_limits<size_t>::max()) ? (check_pos + 2) : 16;
    if (ct_start >= last_frame_len_)
        return;

    constexpr size_t MAX_CT = 128;
    size_t ct_len = last_frame_len_ - ct_start;
    if (ct_len > MAX_CT)
        ct_len = MAX_CT;
    ct_len = (ct_len / 16) * 16;
    if (ct_len < 16)
        return;

    uint8_t ciphertext_buf[MAX_CT];
    uint8_t decrypted_buf[MAX_CT];
    for (size_t i = 0; i < ct_len; i++)
        ciphertext_buf[i] = (uint8_t)last_frame_bytes_[ct_start + i];

    AES_CBC_decrypt_buffer(decrypted_buf, ciphertext_buf, (uint32_t)ct_len,
                           key_.data(), iv_.data());

    ESP_LOGI("APP", "(brummerhoop) AES decrypt ct_start=%u ct_len=%u iv_rebuilt[first16]=...", (unsigned)ct_start, (unsigned)ct_len);

    const uint8_t *decrypted = decrypted_buf;
    size_t decrypted_len = ct_len;





    ESP_LOGI("APP", "(brummerhoop) decrypted payload len=%u", (unsigned)decrypted_len);



    // Log the full decrypted telegram as hex (not only a prefix), similar
    // to the wmbusmeters.org dump style.
    {
        // Decrypted length is already capped by ct_len, so this will print all
        // bytes that were decrypted.
        size_t out_len = decrypted_len * 2 + 1;
        char *buf = (char *)alloca(out_len);
        if (buf) {
            size_t p = 0;
            for (size_t di = 0; di < decrypted_len && p + 2 < out_len; di++) {
                int n = snprintf(&buf[p], out_len - p, "%02X", (unsigned)decrypted[di]);
                if (n <= 0) break;
                p += (size_t)n;
            }
            buf[out_len - 1] = '\0';
            ESP_LOGI("APP", "(brummerhoop) decrypted full hex len=%u=%s", (unsigned)decrypted_len, buf);
        } else {
            ESP_LOGW("APP", "(brummerhoop) decrypted full hex: alloca failed");
        }
    }


    // Search decrypted payload for known patterns:

    // total_m3:  DIF=0x04, VIF=0x13, followed by 4 data bytes (little endian),
    //            where raw is in liters and scale is /1000 => m3.
    // total_backwards: DIF=0x44, VIF=0x93, followed by 2 data bytes (little endian)
    //                   scale /1000.
    auto parse_u32_le_at = [&](size_t pos, bool *ok) -> uint32_t {
        if (pos + 4 > decrypted_len) {

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
        if (pos + 2 > decrypted_len) {

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

    for (size_t i = 0; i + 6 < decrypted_len; i++)

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

