/*
 Copyright (C) 2026

 Simple WaterStar OMS/WMBus driver.
 Based on stripped IZAR RC driver.

 This version:
 - ONLY supports WaterStar meters
 - DOES NOT use Diehl/PRIOS decoding
 - DOES NOT use LFSR decryption
 - Parses standard OMS DV entries directly
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
    di.setName("waterstar");

    di.setDefaultFields(
        "name,id,total_m3,timestamp");

    di.setMeterType(MeterType::WaterMeter);

    di.addLinkMode(LinkMode::T1);
    di.addLinkMode(LinkMode::C1);

    /*
      IMPORTANT:
      Replace MANUFACTURER_WST with the correct
      manufacturer enum if needed.
    */
    di.addDetection(MANUFACTURER_WST, 0x07, -1);

    di.usesProcessContent();

    di.setConstructor([](MeterInfo &mi, DriverInfo &di)
    {
        return std::shared_ptr<Meter>(new Driver(mi, di));
    });
});

Driver::Driver(MeterInfo &mi, DriverInfo &di)
    : MeterCommonImplementation(mi, di)
{
    ESP_LOGI("APP", "(waterstar) Driver loaded");

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
             "(waterstar) telegram received len=%d",
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
    ESP_LOGI("APP", "(waterstar) processContent ENTER");

    if (t == NULL)
    {
        ESP_LOGW("APP", "(waterstar) telegram NULL");
        return;
    }

    /*
      Dump DV entries for debugging.
      This helps identify which key contains
      the total water consumption.
    */
    for (auto &dv : t->dv_entries)
    {
        ESP_LOGI("APP",
                 "(waterstar) DV key=%s value=%s",
                 dv.second.key.c_str(),
                 dv.second.value.c_str());
    }

    /*
      Standard OMS water volume extraction.

      Common VIF:
      0413 = Volume (liters)

      Adjust if your meter uses another key.
    */
    double total_m3 = 0.0;

    bool ok = extractDVdouble(
        &t->dv_entries,
        "0413",
        &total_m3);

    if (ok)
    {
        /*
          Some meters report liters.
          Convert to m³ if needed.
        */

        if (total_m3 > 100000)
        {
            total_m3 /= 1000.0;
        }

        setNumericValue(
            "total",
            Unit::M3,
            total_m3);

        ESP_LOGI("APP",
                 "(waterstar) total_m3=%.3f",
                 total_m3);
    }
    else
    {
        ESP_LOGW("APP",
                 "(waterstar) failed to extract total");
    }

    /*
      Extract meter ID from address.
    */
    if (!t->addresses.empty())
    {
        std::string id = t->addresses.back().id();
        setStringValue("meter_id", id);

        ESP_LOGI("APP",
                 "(waterstar) meter_id=%s",
                 id.c_str());
    }
}

} // namespace
