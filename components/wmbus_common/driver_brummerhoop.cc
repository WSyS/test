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
    di.setName("brummerhoop");

    di.setDefaultFields(
        "name,id,total_m3,timestamp");

    di.setMeterType(MeterType::WaterMeter);

    di.addLinkMode(LinkMode::T1);
    di.addLinkMode(LinkMode::C1);

    di.addDetection(MANUFACTURER_WFT, 0x07, -1);

    di.setConstructor([](MeterInfo &mi, DriverInfo &di)
    {
        return std::shared_ptr<Meter>(new Driver(mi, di));
    });
});

Driver::Driver(MeterInfo &mi, DriverInfo &di)
    : MeterCommonImplementation(mi, di)
{
}

} // namespace