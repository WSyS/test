/*
 WaterStarM minimal driver
*/

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
    addNumericFieldWithExtractor(
        "total",
        "The total water consumption",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::Volume,
        VifScaling::Auto,
        {
            {
                DIFVIFKey("0413"),
                StorageNr(0),
                TariffNr(0),
                SubUnitNr(0)
            }
        });

    addStringFieldWithExtractor(
        "meter_datetime",
        "Meter datetime",
        DEFAULT_PRINT_PROPERTIES,
        {
            {
                DIFVIFKey("046D"),
                StorageNr(0),
                TariffNr(0),
                SubUnitNr(0)
            }
        });
}

} // namespace