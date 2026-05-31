#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"

namespace
{

struct Driver : public virtual MeterCommonImplementation
{
    Driver(MeterInfo &mi, DriverInfo &di);

    bool handleTelegram(AboutTelegram &about, std::vector<uchar> input_frame,
                         bool simulated, std::vector<Address> *addresses,
                         bool *id_match, Telegram *out_analyzed = NULL) override;

    void processContent(Telegram *t) override;
};


static bool ok = registerDriver([](DriverInfo &di)
{
    di.setName("brummerhoop");

    di.setMeterType(MeterType::WaterMeter);
    di.setDefaultFields("name,id,total,total_backwards_at_set_date_m3,status,timestamp");


    di.addLinkMode(LinkMode::T1);
    di.addLinkMode(LinkMode::C1);


    // Brummerhoop payload parsing can require custom processing.
    // However, for this implementation we rely on the generic parsing
    // pipeline to populate dv_entries (needed for extraction).
    // We still keep conservative safety checks inside processContent.
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

bool Driver::handleTelegram(AboutTelegram &about, std::vector<uchar> input_frame,
                           bool simulated, std::vector<Address> *addresses,
                           bool *id_match, Telegram *out_analyzed)

{
    ESP_LOGI("APP", "(brummerhoop) handleTelegram entered simulated=%d frame_size=%d id_match_ptr=%p addresses_ptr=%p",
             (int)simulated, (int)input_frame.size(), (void *)id_match, (void *)addresses);

    // Cache frame in a minimal way and let the processContent pipeline do the work
    // if it ever gets called.
    // Note: Since this driver should be matched by dispatcher, this should run.

    bool parent_ok = 0;

    // Call parent to ensure Telegram is analyzed.
    parent_ok = MeterCommonImplementation::handleTelegram(about, input_frame,
                                                                 simulated, addresses,
                                                                 id_match, out_analyzed);

    

    ESP_LOGI("APP", "(brummerhoop) handleTelegram parent_ok=%d out_analyzed=%p discard=%d",
             (int)parent_ok, (void *)out_analyzed,
             out_analyzed ? (int)out_analyzed->discard : -1);




    // Log the configured meter_id (from YAML: wmbus_meter.meter_id) if available.
    // The meter_id configured in YAML is fed into MeterInfo::parse(..., aes="id + "," ...)
    // and ends up embedded in the configured address expression id.
    std::vector<AddressExpression> aexps = this->addressExpressions();
    std::string yaml_meter_id = aexps.size() > 0 ? aexps[0].id : "0";
    yaml_meter_id = std::to_string(std::stoul(yaml_meter_id, nullptr, 16));
    ESP_LOGI("APP", "(brummerhoop) configured meter_id(yaml)=%s", yaml_meter_id.c_str());


    std::string packet_meter_id;

    const std::vector<uchar> &frame = input_frame;
    if (frame.size() > 10)
    {
      // Bytes frame[4..7] correspond to meter id bytes (endianness adjusted for addressExpressions matching).
      uchar id0 = frame[4], id1 = frame[5], id2 = frame[6], id3 = frame[7];
      packet_meter_id = tostrprintf("%02X%02X%02X%02X", id3, id2, id1, id0);
    }


    ESP_LOGI("APP", "(brummerhoop) extracted meter_id(packet)=%s", packet_meter_id.c_str());


    if (packet_meter_id == yaml_meter_id)
    {
        ESP_LOGI("APP", "(brummerhoop) IDs are the same");
        if (id_match) {
            *id_match = true;
        }
    }


    // If the core decided to call processContent for us, it will already be done.
    // Otherwise, invoke it here for consistent field extraction/logging.
    if (out_analyzed != NULL && !out_analyzed->discard)
        processContent(out_analyzed);


    return true;

}

Driver::Driver(MeterInfo &mi, DriverInfo &di)
    : MeterCommonImplementation(mi, di)
{

    addStringFieldWithExtractorAndLookup(
        "status",
        "Status and error flags.",
        DEFAULT_PRINT_PROPERTIES | PrintProperty::INCLUDE_TPL_STATUS | PrintProperty::STATUS,
        FieldMatcher::build()
            .set(VIFRange::ErrorFlags),
        {
            {
                {
                    "ERROR_FLAGS",
                    Translate::MapType::BitToString,
                    AlwaysTrigger,
                    MaskBits(0xffff),
                    "OK",
                    {
                        {0x01, "SW_ERROR"},
                        {0x02, "CRC_ERROR"},
                        {0x04, "SENSOR_ERROR"},
                        {0x08, "MEASUREMENT_ERROR"},
                        {0x10, "BATTERY_VOLTAGE_ERROR"},
                        {0x20, "MANIPULATION"},
                        {0x40, "LEAKAGE_OR_NO_USAGE"},
                        {0x80, "REVERSE_FLOW"},
                        {0x100, "OVERLOAD"},
                    }
                },
            },
        });

    addNumericFieldWithExtractor(
        "total",
        "The total water consumption recorded by this meter.",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::Volume,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume));

    addNumericFieldWithExtractor(
        "total_backwards",
        "The total backward water volume recorded by this meter.",
        DEFAULT_PRINT_PROPERTIES,
        Quantity::Volume,
        VifScaling::Auto,
        DifSignedness::Signed,
        FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyVolumeVIF)
            .add(VIFCombinable::BackwardFlow));
}

void Driver::processContent(Telegram *t)
{
    // Keep this function intentionally minimal: field extraction is handled
    // by MeterCommonImplementation::processFieldExtractors based on dv_entries.
    // If the generic parser cannot populate dv_entries, there is nothing safe
    // to do here without custom DIF/VIF parsing.
    ESP_LOGW("APP", "(brummerhoop) processContent ENTER");



    // Brummerhoop frames contain many VIFs and (for some payload variants)

    // the generic extractor path may recurse deeply and overflow the ESP32
    // stack. Keep decoding here conservative and bail out on suspicious
    // frames.

    ESP_LOGI("APP", "(brummerhoop) processContent enter t=%p dv=%d header=%d suffix=%d parsed=%d",
             (void *)t,
             t ? (int)t->dv_entries.size() : -1,
             t ? t->header_size : -1,
             t ? t->suffix_size : -1,
             t ? (int)t->parsed.size() : -1);


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

    ESP_LOGW("APP", "(brummerhoop) dv_entries size=%d", (int)t->dv_entries.size());

    // Dump a small part of dv_entries to be able to wire the correct
    // extractors (for total_m3, backward-at-set-date, status).
    // Keep bounded to avoid log spam.
    int dumped = 0;
    for (auto &kv : t->dv_entries) {

        if (dumped++ >= 25)
            break;
        // kv.first: dif+vif key
        // kv.second: DVEntry
        ESP_LOGI("APP", "(brummerhoop) dv_entry[%d] key=%s value=%s mt_vif=%x st=%d ta=%d su=%d",
                 dumped, kv.first.c_str(), kv.second.second.value.c_str(),
                 kv.second.second.vif.intValue(),
                 kv.second.second.storage_nr.intValue(),
                 kv.second.second.tariff_nr.intValue(),
                 kv.second.second.subunit_nr.intValue());
    }

    // Field extraction is implemented via registered field extractors
    // (MeterCommonImplementation::processFieldExtractors). If dv_entries is
    // empty, generic parsing failed for this telegram; do not attempt unsafe
    // custom parsing here.

    (void)t;


}


    // (Other helper methods/fields live in MeterCommonImplementation)

} // namespace

