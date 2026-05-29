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
};

static bool ok = registerDriver([](DriverInfo &di)
{
    di.setName("brummerhoop");

    di.setDefaultFields(
        "name,id,total_m3,meter_datetime,timestamp");

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
        DifSignedness::Unsigned,
        FieldMatcher::build()
            .set(DifVifKey("0413"))
            .set(StorageNr(0))
            .set(TariffNr(0))
            .set(SubUnitNr(0)),
        Unit::M3);

    addStringFieldWithExtractor(
        "meter_datetime",
        "Meter datetime",
        DEFAULT_PRINT_PROPERTIES,
        FieldMatcher::build()
            .set(DifVifKey("046D"))
            .set(StorageNr(0))
            .set(TariffNr(0))
            .set(SubUnitNr(0)));
}

} // namespace