/*
 Simple WaterStar / Brummerhoop OMS driver
*/

#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"

namespace
{

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
};

static bool ok = registerDriver([](DriverInfo &di)
{
    di.setName("brummerhoop");

    di.setDefaultFields(
        "name,id,total_m3,timestamp");

    di.setMeterType(MeterType::WaterMeter);

    di.addLinkMode(LinkMode::T1);
    di.addLinkMode(LinkMode::C1);

    /*
      Use existing manufacturer enum from your framework.
      Replace later if needed.
    */
    di.addDetection(MANUFACTURER_WFT, 0x07, -1);

    di.usesProcessContent();

    di.setConstructor([](MeterInfo &mi, DriverInfo &di)
    {
        return std::shared_ptr<Meter>(new Driver(mi, di));
    });
});

Driver::Driver(MeterInfo &mi, DriverInfo &di)
    : MeterCommonImplementation(mi, di)
{
    ESP_LOGI("APP", "(brummerhoop) Driver loaded");

    addNumericField(
        "total",
        Quantity::Volume,
        DEFAULT_PRINT_PROPERTIES,
        "Total volume");

    addStringField(
        "meter_id",
        "Meter ID",
        DEFAULT_PRINT_PROPERTIES);
}

bool Driver::handleTelegram(AboutTelegram &about,
                            std::vector<uchar> input_frame,
                            bool simulated,
                            std::vector<Address> *addresses,
                            bool *id_match,
                            Telegram *out_analyzed)
{
    ESP_LOGI("APP",
             "(brummerhoop) telegram len=%d",
             (int)input_frame.size());

    return MeterCommonImplementation::handleTelegram(
        about,
        input_frame,
        simulated,
        addresses,
        id_match,
        out_analyzed);
}

void Driver::processContent(Telegram *t)
{
    ESP_LOGI("APP", "(brummerhoop) processContent ENTER");

    if (t == NULL)
    {
        ESP_LOGW("APP", "(brummerhoop) telegram NULL");
        return;
    }

    /*
      Debug DV entries
    */
    for (auto &dv : t->dv_entries)
    {
        ESP_LOGI("APP",
                 "(brummerhoop) DV key=%s",
                 dv.first.c_str());
    }

    /*
      Extract total volume
    */
    double total = 0.0;
    int offset = 0;

    bool found = extractDVdouble(
        &t->dv_entries,
        "0413",
        &offset,
        &total);

    if (found)
    {
        /*
          Convert liters -> m3 if required
        */
        if (total > 100000)
        {
            total = total / 1000.0;
        }

        setNumericValue(
            "total",
            Unit::M3,
            total);

        ESP_LOGI("APP",
                 "(brummerhoop) total_m3=%.3f",
                 total);
    }
    else
    {
        ESP_LOGW("APP",
                 "(brummerhoop) no volume found");
    }

    /*
      Meter ID
    */
    if (!t->addresses.empty())
    {
        std::string id = t->addresses.back().id;
        setStringValue("meter_id", id);

        ESP_LOGI("APP",
                 "(brummerhoop) meter_id=%s",
                 id.c_str());
    }
}

} // namespace