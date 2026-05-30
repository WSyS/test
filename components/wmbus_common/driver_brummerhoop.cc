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

void Driver::processContent(Telegram *t)
{
    // Brummerhoop frames contain many VIFs and (for some payload variants)
    // the generic extractor path may recurse deeply and overflow the ESP32
    // stack. Keep decoding here conservative and bail out on suspicious
    // frames.

    ESP_LOGI("APP", "(brummerhoop) processContent enter t=%p dv=%d header=%d suffix=%d",
             (void *)t, t ? (int)t->dv_entries.size() : -1,
             t ? t->header_size : -1, t ? t->suffix_size : -1);

    if (t == NULL)
        return;

    // Heuristic: the problematic telegrams always contain a DIF/VIF pattern
    // around entries we saw in the logs (e.g. DIF 0x046D + several 0x0413).
    // If the payload ends up with an unexpectedly high amount of dv_entries
    // and/or contains known bad VIFs, discard to prevent stack overflow.
    if (t->dv_entries.size() > 40) {
        ESP_LOGW("APP", "(brummerhoop) discarding: too many dv_entries=%d",
                 (int)t->dv_entries.size());
        t->discard = true;
        return;
    }

    // Additional conservative rule: if the decoded format is clearly not a
    // compact profile for this meter, discard.
    // (We rely on dv_entries extraction already happened; we only avoid
    // further heavy processing.)

    // For now: do not attempt custom extraction; just keep the driver from
    // touching the problematic path further.
}

} // namespace
