
#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"

namespace
{

static bool hexToBytes16(const std::string &hex, std::array<uint8_t, 16> &out)
{
    if (hex.size() != 32)
        return false;

    for (size_t i = 0; i < 16; i++)
    {
        out[i] = (uint8_t)strtoul(hex.substr(i * 2, 2).c_str(), nullptr, 16);
    }

    return true;
}

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
    di.addDetection(MANUFACTURER_DWZ, 0x07, 0x00);
    di.addDetection(MANUFACTURER_DWZ, 0x07, 0x02);

    di.setConstructor([](MeterInfo &mi, DriverInfo &di)
    {
        return std::shared_ptr<Meter>(new Driver(mi, di));
    });
});

bool Driver::handleTelegram(AboutTelegram &about, std::vector<uchar> input_frame,
                           bool simulated, std::vector<Address> *addresses,
                           bool *id_match, Telegram *out_analyzed)
{
    last_frame_ = input_frame;

    bool parent_ok = MeterCommonImplementation::handleTelegram(
        about, input_frame, simulated, addresses, id_match, out_analyzed);

    if (out_analyzed && !out_analyzed->discard)
        processContent(out_analyzed);

    return parent_ok;
}

Driver::Driver(MeterInfo &mi, DriverInfo &di)
    : MeterCommonImplementation(mi, di)
{
    addStringFieldWithExtractorAndLookup(
        "status",
        "Status and error flags.",
        DEFAULT_PRINT_PROPERTIES | PrintProperty::STATUS,
        FieldMatcher::build().set(VIFRange::ErrorFlags),
        {{
            {"ERROR_FLAGS", Translate::MapType::BitToString,
             AlwaysTrigger, MaskBits(0xffff),
             "OK",
             {
                 {0x01, "SW_ERROR"},
                 {0x02, "CRC_ERROR"},
                 {0x04, "SENSOR_ERROR"},
                 {0x08, "MEASUREMENT_ERROR"},
                 {0x10, "BATTERY_ERROR"},
                 {0x20, "MANIPULATION"},
                 {0x40, "LEAKAGE"},
                 {0x80, "REVERSE_FLOW"},
             }}
        }});

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
            .set(VIFRange::AnyVolumeVIF)
            .add(VIFCombinable::BackwardFlow));
}

void Driver::processContent(Telegram *t)
{
    if (!t) return;

    ESP_LOGW("APP", "(brummerhoop) processContent ENTER");

    if (t->dv_entries.empty())
    {
        ESP_LOGW("APP", "(brummerhoop) AES fallback activated");

        const std::vector<uchar> &frame = last_frame_;

        if (frame.size() < 32)
            return;

        std::vector<AddressExpression> aexps = this->addressExpressions();
        std::string key_hex = (aexps.size() > 0) ? aexps[0].id : "";

        std::array<uint8_t, 16> key{};
        if (!hexToBytes16(key_hex, key))
        {
            ESP_LOGE("APP", "(brummerhoop) invalid AES key");
            return;
        }

        std::array<uint8_t, 16> iv{};
        memcpy(iv.data(), frame.data(), 16);

        std::vector<uint8_t> ciphertext(frame.begin() + 16, frame.end());
        std::vector<uint8_t> decrypted(ciphertext.size());

        if (!aes_cbc_decrypt(ciphertext, key, iv, decrypted))
        {
            ESP_LOGE("APP", "(brummerhoop) AES decrypt failed");
            return;
        }

        for (size_t i = 0; i + 6 < decrypted.size(); i++)
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

        return;
    }

    this->processFieldExtractors(t);
}

} // namespace
