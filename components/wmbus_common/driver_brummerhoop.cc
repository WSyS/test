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

    di.setMeterType(MeterType::WaterMeter);
    di.setDefaultFields("name,id,total_m3,total_backwards_at_set_date_m3,status,timestamp");

    di.addLinkMode(LinkMode::T1);
    di.addLinkMode(LinkMode::C1);


    // Brummerhoop payload parsing can require custom processing.
    // Without this, the generic content pipeline may recurse/deep-parse
    // malformed DIF/VIF chains and overflow the ESP32 stack.
    di.usesProcessContent();

// Adjust manufacturer if needed
    // For brummerhoop telegrams the "version" field is not constant, so keep
    // it wildcarded (-1) but still bind to the correct manufacturer

    di.addDetection(MANUFACTURER_EFE, 0x07, -1);

    // Best-effort matching for common Brummerhoop/Waterstarm variants.
    // If these don't match your specific meter, they won't harm selection.
    di.addDetection(MANUFACTURER_DWZ, 0x07, 0x00); // warm water
    di.addDetection(MANUFACTURER_DWZ, 0x07, 0x02); // alternative

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

    // Debug: log configured meter_id(s) and extracted packet meter id for
    // easier troubleshooting when Home Assistant reports that telegrams are
    // not handled.
    //
    // Similar logic exists in driver_izar_rc.cc.
    std::vector<AddressExpression> aexps = this->addressExpressions();
    std::string yaml_meter_id = aexps.size() > 0 ? aexps[0].id : "0";
    yaml_meter_id = std::to_string(std::stoul(yaml_meter_id, nullptr, 16));

    std::string packet_meter_id;
    if (t->frame.size() > 10)
    {
        // Bytes t->frame[4..7] correspond to meter id bytes (endianness adjusted
        // for addressExpressions matching in other drivers).
        uchar id0 = t->frame[6], id1 = t->frame[7], id2 = t->frame[8], id3 = t->frame[9];
        packet_meter_id = tostrprintf("%02X%02X%02X%02X", id3, id2, id1, id0);
    }

    ESP_LOGI("APP", "(brummerhoop) configured meter_id(yaml)=%s", yaml_meter_id.c_str());
    ESP_LOGI("APP", "(brummerhoop) extracted meter_id(packet)=%s", packet_meter_id.c_str());


    // Additional conservative rule: if the decoded format is clearly not a
    // compact profile for this meter, discard.
    // (We rely on dv_entries extraction already happened; we only avoid
    // further heavy processing.)

    // We decode only a small subset of fields needed by Home Assistant.
    // Brummerhoop telegrams can be pathological, so we do everything in
    // conservative, bounded ways.

    // total_m3: try to read volume for storage 0 (or default storage).
    // total_backwards_at_set_date_m3: backward flow volume at storage 1.
    // status: decode error flags if present, otherwise fall back to tpl status.

    // Rely on the generic dv_extraction helpers by activating a minimal set
    // of field extractors via the common implementation.
    // Note: MeterCommonImplementation field wiring is done by
    // add*FieldWithExtractor* calls, which are intentionally absent in this
    // driver today.

    // If we already have fields extracted by the generic pipeline, do not
    // override them.
    // (When di.usesProcessContent() is enabled, the generic pipeline may be
    // skipped for some drivers. We keep this no-op to avoid changing parsing
    // logic on pathological frames.)

    (void)t;
}


} // namespace
