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

    // AES helper expects std::vector<uchar>.
    std::vector<uchar> key_vec(16);
    memcpy(key_vec.data(), key_.data(), 16);




    // Log key hex for debugging.
    char key_hex[33];
    for (size_t i = 0; i < 16; i++)
        sprintf(&key_hex[i * 2], "%02X", key_.data()[i]);
    key_hex[32] = '\0';
    ESP_LOGI("APP", "(brummerhoop) AES key loaded (len=32) %s", key_hex);

    // IV for TPL AES-CBC must be built the same way as decrypt_TPL_AES_CBC_IV()
    // in wmbus_utils.cc.
    // In our fallback mode, generic parsing might not have populated t->dll_a,
    // so we reconstruct dll_mfct_b + dll_a from the trimmed frame bytes.

    // We must locate the ciphertext start by parsing TPL-CFG.
    // In OMS/wM-Bus the bytes after TPL-CFG (3025) are the AES-CBC ciphertext.
    // Do NOT search for 0x2F2F in the trimmed frame; 2F2F is expected in plaintext after decryption.

    size_t tpl_cfg_pos = std::numeric_limits<size_t>::max();
    for (size_t p = 0; p + 1 < last_frame_len_; ++p) {
        if (last_frame_bytes_[p] == 0x30 && last_frame_bytes_[p + 1] == 0x25) {
            tpl_cfg_pos = p;
            break;
        }
    }

    if (tpl_cfg_pos == std::numeric_limits<size_t>::max()) {
        // Dump trimmed frame A for diagnostics.
        std::string hx;
        hx.reserve(last_frame_len_ * 2);
        for (size_t i = 0; i < last_frame_len_; ++i) {
            char b[3];
            sprintf(b, "%02X", last_frame_bytes_[i]);
            hx += b;
        }
        ESP_LOGE("APP", "(brummerhoop) AES fallback: TPL-CFG 30 25 not found in trimmed frame. last_frame_len_=%u trimmed_hex=%s",
                 (unsigned)last_frame_len_, hx.c_str());
        return;
    }

    // Skip TPL-CFG (2 bytes: 30 25).
    size_t ct_start_base = tpl_cfg_pos + 2;
    if (ct_start_base >= last_frame_len_)
        return;

    constexpr size_t MAX_CT = 128;
    size_t ct_len_base = last_frame_len_ - ct_start_base;
    if (ct_len_base > MAX_CT)
        ct_len_base = MAX_CT;

    // ct_len must be a multiple of 16 for AES-CBC.
    ct_len_base = ((ct_len_base / 16) * 16);
    if (ct_len_base < 16)
        return;

    // Prefer decrypting more blocks to include both forwards and backwards totals.
    // Some telegrams only show the DIF/VIF for `total` beyond the first 3 blocks.
    constexpr size_t TARGET_CT = 96;
    size_t ct_len = ct_len_base;
    if (ct_len > TARGET_CT)
        ct_len = TARGET_CT;

    // IV for OMS Mode 5 is expected to be:
    // M(2 bytes) | Address(4 bytes) | Version(DeviceType)(2 bytes) | ACC x8
    // ACC is the access number and must be repeated 8x in the IV.
    // In this driver we don't have a full OMS parser, but we can reconstruct the IV
    // from the trimmed frame prefix (best-effort):
    // M(2)=frame[0..1], Address(4)=frame[2..5], Version/DevType(2)=frame[6..7].

    // We will no longer guess ACC from a fixed index; instead we brute-force ACC values
    // in a small candidate range derived from the trimmed payload.

    constexpr size_t SWEEP_MAX_SHIFT = 16; // try ct_start_base + 0..15

    std::array<uint8_t, 16> iv_best_{};
    uint8_t best_iv_acc = 0;
    if (last_frame_len_ < 16) {
        ESP_LOGE("APP", "(brummerhoop) AES fallback: trimmed frame too small for IV build");
        return;
    }

    // Build the constant IV prefix (M+Address+Version/DevType). ACC is brute-forced.
    uint8_t m0 = last_frame_bytes_[0];
    uint8_t m1 = last_frame_bytes_[1];
    uint8_t a0 = last_frame_bytes_[2];
    uint8_t a1 = last_frame_bytes_[3];
    uint8_t a2 = last_frame_bytes_[4];
    uint8_t a3 = last_frame_bytes_[5];
    uint8_t ver0 = last_frame_bytes_[6];
    uint8_t dev0 = last_frame_bytes_[7];

    // Offset sweep (ciphertext start) remains.
    uint8_t ciphertext_buf[MAX_CT];
    uint8_t decrypted_buf[MAX_CT];

    // ACC brute-force search range.
    // Access numbers observed are typically small (often 0x00..0x7F). We'll try 0..127.
    constexpr uint8_t ACC_MIN = 0;
    constexpr uint8_t ACC_MAX = 127;

    int best_score = -1;
    size_t best_ct_start = ct_start_base;

    for (uint8_t acc = ACC_MIN; acc <= ACC_MAX; ++acc) {
        std::array<uint8_t, 16> iv_{};
        iv_[0] = m0;
        iv_[1] = m1;
        iv_[2] = a0;
        iv_[3] = a1;
        iv_[4] = a2;
        iv_[5] = a3;
        iv_[6] = ver0;
        iv_[7] = dev0;
        for (int j = 0; j < 8; ++j) {
            iv_[8 + j] = acc;
        }

        // Decrypt each ciphertext-start candidate with this IV and score markers.
        for (size_t shift = 0; shift <= SWEEP_MAX_SHIFT; ++shift) {
            size_t ct_start = ct_start_base + shift;
            if (ct_start + ct_len > last_frame_len_)
                break;

            memcpy(ciphertext_buf, &last_frame_bytes_[ct_start], ct_len);

            AES_CBC_decrypt_buffer(decrypted_buf, ciphertext_buf, (uint32_t)ct_len,
                                   safeButUnsafeVectorPtr(key_vec), iv_.data());

            int score = 0;
            for (size_t i = 0; i + 5 < ct_len; ++i) {
                if (decrypted_buf[i] == 0x04 && decrypted_buf[i + 1] == 0x13)
                    score += 3;
                if (decrypted_buf[i] == 0x44 && decrypted_buf[i + 1] == 0x93)
                    score += 2;
            }

            if (score > best_score) {
                best_score = score;
                best_ct_start = ct_start;
                best_iv_acc = acc;
                // keep best decrypted in decrypted_buf for later logging
            }

            ESP_LOGI("APP", "(brummerhoop) AES fallback sweep acc=%u shift=%u score=%d ct_start=%u", (unsigned)acc, (unsigned)shift, score, (unsigned)ct_start);
        }
    }

    ESP_LOGE("APP", "(brummerhoop) AES fallback best acc=%u best_ct_start=%u best_score=%d ct_len=%u",
             (unsigned)best_iv_acc, (unsigned)best_ct_start, best_score, (unsigned)ct_len);

    // Decrypt once more using best IV/offset so marker scan below sees the best plaintext.
    std::array<uint8_t, 16> iv_best_{};
    iv_best_[0] = m0;
    iv_best_[1] = m1;
    iv_best_[2] = a0;
    iv_best_[3] = a1;
    iv_best_[4] = a2;
    iv_best_[5] = a3;
    iv_best_[6] = ver0;
    iv_best_[7] = dev0;
    for (int j = 0; j < 8; ++j) {
        iv_best_[8 + j] = best_iv_acc;
    }

    memcpy(ciphertext_buf, &last_frame_bytes_[best_ct_start], ct_len);
    AES_CBC_decrypt_buffer(decrypted_buf, ciphertext_buf, (uint32_t)ct_len,
                           safeButUnsafeVectorPtr(key_vec), iv_best_.data());

    // From here on, decrypted_buf is already the best candidate plaintext.

    // (ciphertext head/decrypted hex logging + marker scan remain unchanged below)

    int best_score = -1;
    size_t best_ct_start = ct_start_base;

    for (size_t shift = 0; shift <= SWEEP_MAX_SHIFT; ++shift) {
        size_t ct_start = ct_start_base + shift;
        if (ct_start + ct_len > last_frame_len_)
            break;

        memcpy(ciphertext_buf, &last_frame_bytes_[ct_start], ct_len);

        AES_CBC_decrypt_buffer(decrypted_buf, ciphertext_buf, (uint32_t)ct_len,
                               safeButUnsafeVectorPtr(key_vec), iv_.data());

        // Score how many expected markers exist.
        int score = 0;
        for (size_t i = 0; i + 5 < ct_len; ++i) {
            if (decrypted_buf[i] == 0x04 && decrypted_buf[i + 1] == 0x13)
                score += 3;
            if (decrypted_buf[i] == 0x44 && decrypted_buf[i + 1] == 0x93)
                score += 2;
        }

        if (score > best_score) {
            best_score = score;
            best_ct_start = ct_start;
        }

        ESP_LOGI("APP", "(brummerhoop) AES fallback sweep shift=%u score=%d ct_start=%u", (unsigned)shift, score, (unsigned)ct_start);
    }

    // Decrypt once more using the best offset so the marker logs below match.
    memcpy(ciphertext_buf, &last_frame_bytes_[best_ct_start], ct_len);
    AES_CBC_decrypt_buffer(decrypted_buf, ciphertext_buf, (uint32_t)ct_len,
                           safeButUnsafeVectorPtr(key_vec), iv_.data());

    ESP_LOGE("APP", "(brummerhoop) AES fallback best_ct_start=%u best_score=%d ct_len=%u",
             (unsigned)best_ct_start, best_score, (unsigned)ct_len);



    // Dump ciphertext head to avoid huge logs.
    {
        size_t dump_len = ct_len < 64 ? ct_len : 64;
        std::string hx;
        hx.reserve(dump_len * 2);
        for (size_t i = 0; i < dump_len; ++i) {
            char b[3];
            sprintf(b, "%02X", ciphertext_buf[i]);
            hx += b;
        }
        ESP_LOGI("APP", "(brummerhoop) AES fallback debug: ciphertext head=%s", hx.c_str());
    }

    // Log full decrypted payload as hex.
    {
        std::string dhx;
        dhx.reserve(ct_len * 2);
        for (size_t i = 0; i < ct_len; ++i) {
            char b[3];
            sprintf(b, "%02X", decrypted_buf[i]);
            dhx += b;
        }
        ESP_LOGE("APP", "(brummerhoop) AES fallback decrypted full hex len=%u: %s",
                 (unsigned)ct_len, dhx.c_str());
    }


    // Dump ciphertext (first 64 bytes) to avoid huge logs.
    {
        size_t dump_len = ct_len < 64 ? ct_len : 64;
        std::string hx;
        hx.reserve(dump_len * 2);
        for (size_t i = 0; i < dump_len; ++i) {
            char b[3];
            sprintf(b, "%02X", ciphertext_buf[i]);
            hx += b;
        }
        ESP_LOGI("APP", "(brummerhoop) AES fallback debug: ciphertext head=%s", hx.c_str());
    }

    // NOTE: legacy extra decrypt/log block removed.

    // Log full decrypted payload as hex.

    {
        std::string dhx;
        dhx.reserve(ct_len * 2);
        for (size_t i = 0; i < ct_len; ++i) {
            char b[3];
            sprintf(b, "%02X", decrypted_buf[i]);
            dhx += b;
        }
        ESP_LOGE("APP", "(brummerhoop) AES fallback decrypted full hex len=%u: %s",
                 (unsigned)ct_len, dhx.c_str());
    }

    // Scan for DIF/VIF markers 04 13 and 44 93.
    bool found_total = false;
    bool found_back = false;
    for (size_t i = 0; i + 5 < ct_len; ++i) {
        if (decrypted_buf[i] == 0x04 && decrypted_buf[i + 1] == 0x13) {
            found_total = true;
            uint32_t raw = decrypted_buf[i + 2] |
                             (uint32_t(decrypted_buf[i + 3]) << 8) |
                             (uint32_t(decrypted_buf[i + 4]) << 16) |
                             (uint32_t(decrypted_buf[i + 5]) << 24);
            double total_m3 = raw / 1000.0;
            ESP_LOGE("APP", "(brummerhoop) AES fallback found total marker at pos=%u raw=%u total_m3=%f",
                     (unsigned)i, (unsigned)raw, total_m3);
            if (!this->hasNumericValue(this->findFieldInfo("total", Quantity::Volume))) {
                setNumericValue("total", Unit::M3, total_m3);
            } else {
                setNumericValue("total", Unit::M3, total_m3);
            }
        }
        if (decrypted_buf[i] == 0x44 && decrypted_buf[i + 1] == 0x93) {
            found_back = true;
            if (i + 3 < ct_len) {
                uint16_t raw = decrypted_buf[i + 2] |
                                 (uint16_t(decrypted_buf[i + 3]) << 8);
                double total_back_m3 = raw / 1000.0;
                ESP_LOGE("APP", "(brummerhoop) AES fallback found total_back marker at pos=%u raw=%u total_back_m3=%f",
                         (unsigned)i, (unsigned)raw, total_back_m3);
                setNumericValue("total_backwards", Unit::M3, total_back_m3);
            }
        }
    }

    if (!found_total)
        ESP_LOGE("APP", "(brummerhoop) AES fallback debug: DIF/VIF total marker 04 13 not found in decrypted payload");
    if (!found_back)
        ESP_LOGE("APP", "(brummerhoop) AES fallback debug: DIF/VIF total_back marker 44 93 not found in decrypted payload");

    // Done.
    return;
}

} // namespace


