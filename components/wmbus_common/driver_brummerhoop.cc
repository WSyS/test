/*
 WaterStarM driver
*/

#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"

namespace
{

struct Driver : public virtual MeterCommonImplementation
{
    Driver(MeterInfo &mi, DriverInfo &di);

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
    addNumericField(
        "total",
        Quantity::Volume,
        DEFAULT_PRINT_PROPERTIES,
        "Total");
}

void Driver::processContent(Telegram *t)
{
    if (t == NULL)
        return;

    /*
      Current total volume:
      DIF/VIF 0413
      storage=0
    */

    double total_m3 = 0.0;
    int offset = 0;

    bool ok = extractDVdouble(
        &t->dv_entries,
        "0413",
        &offset,
        &total_m3);

    if (ok)
    {
        setNumericValue(
            "total",
            Unit::M3,
            total_m3);

        ESP_LOGI("APP",
                 "(waterstarm) total_m3=%.3f",
                 total_m3);
    }
}

} // namespace