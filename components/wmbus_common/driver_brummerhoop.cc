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
    di.setDefaultFields("name,id,meter_datetime,set_date,consumption_at_set_date_m3,total,total_backwards_at_set_date_m3,status,rssi");

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


    if (out_analyzed != nullptr && !out_analyzed->discard)
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

    addNumericFieldWithExtractor(
        "meter_datetime",
        "The meter date and time (billing readout base).",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::PointInTime,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::DateTime));

    addNumericFieldWithExtractor(
        "set_date",
        "The most recent billing period date.",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::PointInTime,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Date)
            .set(StorageNr(1)),
        Unit::DateLT);

    addNumericFieldWithExtractor(
        "consumption_at_set_date_m3",
        "The total water consumption at the most recent billing period date.",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::Volume,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume)
            .set(StorageNr(1)));

}

void Driver::processContent(Telegram *t)
{
    // Definitive safety: the AES fallback must never dereference t when it is NULL.
    if (t == nullptr)
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


    ESP_LOGI("APP", "(brummerhoop) rssi=%d",
          t->about.rssi_dbm);

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
    //
    // Earlier versions of this fallback reconstructed the IV from trimmed
    // raw bytes, but that is brittle because byte offsets differ between
    // frame A/B trimming and different TPL formats.
    //
    // Here we use the parsed Telegram fields (t->tpl_* when available,
    // otherwise t->dll_*), matching decrypt_TPL_AES_CBC_IV().

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

    // OMS AES-CBC payload starts directly after TPL-CFG (30 25)
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

    // Prefer decrypting more blocks to include forwards and backwards totals.
    // Forward total may live in later blocks, so don't truncate too early.
    constexpr size_t TARGET_CT = 128; // was 96
    size_t ct_len = ct_len_base;
    if (ct_len > TARGET_CT)
        ct_len = TARGET_CT;

    // Build IV directly from raw telegram bytes.
    // Format:
    // M-field(2) + A-field(6) + ACC repeated 8 times
    std::array<uint8_t, 16> iv_{};


    if (last_frame_len_ < 16) {
        ESP_LOGE("APP", "(brummerhoop) AES fallback: trimmed frame too small for IV build");
        return;
    }

    // Manufacturer
    iv_[0] = last_frame_bytes_[2];
    iv_[1] = last_frame_bytes_[3];

    for (int j = 0; j < 6; j++)
    {
        iv_[2 + j] = last_frame_bytes_[4 + j];
    }

    // ACC (access number / access count) is 1 byte and must be taken from
    // the protocol header (TPL ACC field), not from a fixed byte offset in
    // the trimmed frame.
    //
    // In our trimmed frame cache we may not know the exact position of
    // TPL-ACC, but using a wrong ACC breaks the IV and corrupts the first
    // CBC block while leaving later blocks plausible.
    //
    // Heuristic fallback: search backwards from the end for a TPL short header
    // that matches our configuration (TPL-CI 0x7A/0x72/0x78/0x79/0x7B variants).
    // If not found, keep ACC=0 (worst case: first block wrong).

    uint8_t acc = 0;

    for (size_t p = 0; p + 2 < last_frame_len_; ++p) {
        // Potential TPL CI values for short header are typically 0x7A/0x78/0x79/0x72.
        if (last_frame_bytes_[p] == 0x7A || last_frame_bytes_[p] == 0x72 ||
            last_frame_bytes_[p] == 0x78 || last_frame_bytes_[p] == 0x79) {
            // In short header parsing, tpl_acc is the first byte after tpl_ci.
            // Our trimmed cache includes that field.
            //acc = last_frame_bytes_[14];
            acc = last_frame_bytes_[p + 1];

            break;
        }
    }

    for (int j = 0; j < 8; ++j)
        iv_[8 + j] = acc;



    char iv_hex[33];
    for (size_t i = 0; i < 16; i++)
        sprintf(&iv_hex[i * 2], "%02X", iv_[i]);
    iv_hex[32] = '\0';

    ESP_LOGE("APP", "(brummerhoop) AES fallback using raw-frame IV=%s acc=%u", iv_hex, (unsigned)acc);



    // Offset sweep: ciphertext start may be off by a few bytes depending on
    // how trimmed frames are constructed.
    // Expand search window.
    constexpr size_t SWEEP_MAX_SHIFT = 64;


    uint8_t ciphertext_buf[MAX_CT];
    uint8_t decrypted_buf[MAX_CT];

    int best_score = -1;
    size_t best_ct_start = ct_start_base;

    for (size_t shift = 0; shift <= SWEEP_MAX_SHIFT; ++shift) {
        size_t ct_start = ct_start_base + shift;
        if (ct_start + ct_len > last_frame_len_)
            break;

        memcpy(ciphertext_buf, &last_frame_bytes_[ct_start], ct_len);

        AES_CBC_decrypt_buffer(decrypted_buf, ciphertext_buf, (uint32_t)ct_len,
                               safeButUnsafeVectorPtr(key_vec), iv_.data());

        // bonus score for 2F2F trailer
        int score = 0;
        if (ct_len >= 4)
        {
            if (decrypted_buf[ct_len-2] == 0x2F &&
                decrypted_buf[ct_len-1] == 0x2F)
                score += 10;
        }


        // Score how many expected markers exist.
        for (size_t i = 0; i + 5 < ct_len; ++i) {
            if (decrypted_buf[i] == 0x04 && decrypted_buf[i + 1] == 0x13)
                score += 3;
            if (decrypted_buf[i] == 0x44 && decrypted_buf[i + 1] == 0x93)
                score += 2;
            if (decrypted_buf[i] == 0x04 && decrypted_buf[i + 1] == 0x6D)
                score += 2;
            if (decrypted_buf[i] == 0x42 && decrypted_buf[i + 1] == 0x6C)
                score += 2;
            if (decrypted_buf[i] == 0x04 && decrypted_buf[i + 1] == 0x13)
                score += 1;
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

    // Scan for DIF/VIF markers.

    bool found_total = false;
    bool found_meter_datetime = false;
    bool found_set_date = false;
    bool found_consumption_at_set_date = false;

    bool found_back = false;

    for (size_t i = 0; i + 5 < ct_len; ++i) {
        // meter_datetime: 6D (VIFDateTime) is decoded as a 4-byte DV after DIF=04.
        if (decrypted_buf[i] == 0x04 && decrypted_buf[i + 1] == 0x6D) {
            found_meter_datetime = true;
            // Payload as per your example: "meter_datetime":"2026-06-03 23:24"
            // Decode uses the same DV date extraction layout as dvparser:
            // - 4 bytes: YYYY MMMM? + YYY DDDDD?? time bits are encoded in the last byte nibble.
            // Here we only extract the human-readable fields directly from the example format:
            // bytes: [b0 b1 b2 b3] little-endian in the DV hex, so treat as big-endian order used by dvparser.
            uint8_t b0 = decrypted_buf[i + 2];
            uint8_t b1 = decrypted_buf[i + 3];
            uint8_t b2 = decrypted_buf[i + 4];
            uint8_t b3 = decrypted_buf[i + 5];
            // Best-effort: dvparser's extractDate/extractTime expects hi/lo ordering.
            // For consistency with dvparser's DVEntry::extractDate for 4-byte values:
            // it calls extractDate(v[3], v[2]) and extractTime(v[1], v[0]).
            // Therefore map our decrypted bytes so that v[0]=b0, v[1]=b1, v[2]=b2, v[3]=b3.
            // DateTime encoding in your telegram extract (VIF 6D):
            // byte[0..3] contain date+time fields packed similarly to dvparser's
            // extractDate(hi,lo) and extractTime(hi,lo) used for 4-byte DV dates.
            // dvparser's DVEntry::extractDate for v.size()==4 does:
            //   extractDate(v[3], v[2]);
            //   extractTime(v[1], v[0]);
            tm tm_out{};
            // dvparser helpers are TU-local in dvparser.cc, so we re-decode using
            // the same bit layouts as extractDate/extractTime.
            // extractDate(hi,lo):
            //   day = lo & 0x1f
            //   month = hi & 0x0f
            //   year1 = (lo & 0xe0) >> 5
            //   year2 = (hi & 0xf0) >> 1
            int day = (b1) & 0x1f;
            int year1 = ((b1) & 0xe0) >> 5;
            int month = (b2) & 0x0f;
            int year2 = ((b2) & 0xf0) >> 1;
            int year = 2000 + year1 + year2;

            int min = (b0) & 0x3f;
            int hour = (b3) & 0x1f;

            bool ok_date = (month <= 12) && (min <= 59) && (hour <= 23);
            if (ok_date) {
                tm_out.tm_mday = day;
                tm_out.tm_mon = month - 1;
                tm_out.tm_year = year - 1900;
                tm_out.tm_min = min;
                tm_out.tm_hour = hour;
                tm_out.tm_isdst = -1;
            }
            if (ok_date) {
                // DateTime decoding skipped in AES fallback.
                // Generic dvparser pipeline should handle meter_datetime when dv_entries are present.
            }
        }

        // set_date: 6C is decoded as "YYYY-MM-DD" (Date G) in a 2-byte DIF after DIF=42.
        // In the example: 42 6C then 2 bytes.
        // set_date: We cannot reliably decode Date here without dvparser helpers.
        // Keep marker detection for AES fallback robustness.
        if (decrypted_buf[i] == 0x42 && decrypted_buf[i + 1] == 0x6C) {
            found_set_date = true;
            if (i + 3 < ct_len) {
                (void)decrypted_buf;
                // Date decoding skipped in AES fallback.
                // Generic dvparser pipeline should decode set_date when dv_entries are present.
            }
        }

        // consumption_at_set_date_m3: 40 13 (volume) followed by 4 bytes.
        if (decrypted_buf[i] == 0x40 && decrypted_buf[i + 1] == 0x13) {
            found_consumption_at_set_date = true;
            if (i + 6 < ct_len) {
                uint32_t raw = decrypted_buf[i + 2] |
                                 (uint32_t(decrypted_buf[i + 3]) << 8) |
                                 (uint32_t(decrypted_buf[i + 4]) << 16) |
                                 (uint32_t(decrypted_buf[i + 5]) << 24);
                double v_m3 = raw / 1000.0;
                setNumericValue("consumption_at_set_date_m3", Unit::M3, v_m3);
            }
        }

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
            if (i + 6 < ct_len) {
                uint16_t raw = decrypted_buf[i + 3] |
                             (uint32_t(decrypted_buf[i + 4]) << 8) |
                             (uint32_t(decrypted_buf[i + 5]) << 16) |
                             (uint32_t(decrypted_buf[i + 6]) << 24);
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




