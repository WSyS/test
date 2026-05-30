#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"

namespace
{

struct Driver : public virtual MeterCommonImplementation
{
    Driver(MeterInfo &mi, DriverInfo &di);
};

static bool ok = registerDriver([](DriverInfo &di)
{
    //di.setName("brummerhoop");

    di.setMeterType(MeterType::WaterMeter);

    di.addLinkMode(LinkMode::T1);
    di.addLinkMode(LinkMode::C1);

    // Brummerhoop payload parsing can require custom processing.
    // Without this, the generic content pipeline may recurse/deep-parse
    // malformed DIF/VIF chains and overflow the ESP32 stack.
    di.usesProcessContent();

    // Adjust manufacturer if needed
    di.addDetection(MANUFACTURER_EFE, 0x07, -1);

    di.setConstructor([](MeterInfo &mi, DriverInfo &di)
    {
        return std::shared_ptr<Meter>(new Driver(mi, di));
    });
});

Driver::Driver(MeterInfo &mi, DriverInfo &di)
    : MeterCommonImplementation(mi, di)
{
    ESP_LOGE("APP",
             "**************** WATERSTARM DRIVER LOADED ****************");
}

} // namespace