/*
 Copyright (C) 2019 Jacek Tomasiak (gpl-3.0-or-later)
 Copyright (C) 2020-2023 Fredrik Öhrström (gpl-3.0-or-later)
 Copyright (C) 2021 Vincent Privat (gpl-3.0-or-later)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"

namespace
{
  /** Contains all the booleans required to store the alarms of a PRIOS device. */
  struct IzarRcAlarms
  {
    bool general_alarm;
    bool leakage_currently;
    bool leakage_previously;
    bool meter_blocked;
    bool back_flow;
    bool underflow;
    bool overflow;
    bool submarine;
    bool sensor_fraud_currently;
    bool sensor_fraud_previously;
    bool mechanical_fraud_currently;
    bool mechanical_fraud_previously;
  };

  struct Driver : public virtual MeterCommonImplementation
  {
    Driver(MeterInfo &mi, DriverInfo &di);

    bool handleTelegram(AboutTelegram &about, std::vector<uchar> input_frame,
                        bool simulated, std::vector<Address> *addresses,
                        bool *id_match, Telegram *out_analyzed = NULL) override;

    void processContent(Telegram *t) override;

  private:
    std::string currentAlarmsText(IzarRcAlarms &alarms);
    std::string previousAlarmsText(IzarRcAlarms &alarms);
    std::vector<uchar> decodePrios(const std::vector<uchar> &origin,
                                   const std::vector<uchar> &frame,
                                   uint32_t key);

    bool extractIzarRcId(const std::vector<uchar> &frame, std::string *id_str);

    std::vector<uint32_t> keys;
    std::vector<uchar> cached_input_frame_;
  };

  static bool ok = registerDriver([](DriverInfo &di)
                                  {
        di.setName("izar_rc");
        di.setDefaultFields("name,id,total_m3,target_m3,current_alarms,remaining_battery_life_y,transmit_period_s,manufacture_year,timestamp");
        di.setMeterType(MeterType::WaterMeter);
        di.addLinkMode(LinkMode::T1);
        di.addLinkMode(LinkMode::C1);
        di.addDetection(MANUFACTURER_DME, 0x07, 0x78);
        di.addDetection(MANUFACTURER_HYD, 0x07, 0x85);
        di.addDetection(MANUFACTURER_SAP, 0x07, 0x00);
        di.addDetection(MANUFACTURER_SAP, 0x04, -1);
        di.addDetection(MANUFACTURER_SAP, 0x15, -1);

        // Ensure processContent() is called for IZAR RC telegrams.
        di.usesProcessContent();

        di.setConstructor([](MeterInfo &mi, DriverInfo &di) { return std::shared_ptr<Meter>(new Driver(mi, di)); }); });

  Driver::Driver(MeterInfo &mi, DriverInfo &di) : MeterCommonImplementation(mi, di)
  {
    ESP_LOGI("APP", "(izar_rc) Driver constructor entered");
    ESP_LOGI("APP", "(izar_rc) Calling debug replaced with LOG");
    ESP_LOGI("APP", "(izar_rc) Driver loaded v2.0");

    initializeDiehlDefaultKeySupport(meterKeys()->confidentiality_key, keys);

    // Debug: log the confidentiality key that drives PRIOS/LFSR decoding.
    // Format: hex string (may be removed after debugging).
    ESP_LOGW("APP", "(izar_rc) confidentiality_key_hex=%s",
             bin2hex(meterKeys()->confidentiality_key).c_str());

    ESP_LOGI("APP", "(izar_rc) keys.size()=%d first key %08x",
             (int)keys.size(), keys.size() > 0 ? keys[0] : 0);

    addStringField("prefix", "Prefix", DEFAULT_PRINT_PROPERTIES);
    addStringField("serial_number", "Serial", DEFAULT_PRINT_PROPERTIES);
    addStringField("manufacture_year", "Year", DEFAULT_PRINT_PROPERTIES);

    // We store totals and target in both units so wrapper selection cannot
    // cause NaN due to unit mismatch.
    addNumericField("total", Quantity::Volume, DEFAULT_PRINT_PROPERTIES, "Total m3");
    addNumericField("target", Quantity::Volume, DEFAULT_PRINT_PROPERTIES, "Target m3");

    addNumericField("remaining_battery_life", Quantity::Time, DEFAULT_PRINT_PROPERTIES,
                    "Battery years", Unit::Year);
    addNumericField("transmit_period", Quantity::Time, DEFAULT_PRINT_PROPERTIES,
                    "TX s", Unit::Second);
    addStringField("current_alarms", "Alarms", DEFAULT_PRINT_PROPERTIES);

    // String fields set by processContent() (see Driver::processContent)
    addStringField("last_month_measure_date", "Last month measure date",
                   DEFAULT_PRINT_PROPERTIES);
    addStringField("previous_alarms", "Previous alarms",
                   DEFAULT_PRINT_PROPERTIES);

    ESP_LOGI("APP", "(izar_rc) Driver constructor exit, all fields added");
  }

  std::string Driver::currentAlarmsText(IzarRcAlarms &alarms)
  {
    std::string s;
    if (alarms.leakage_currently)
    {
      s.append("leakage,");
    }
    if (alarms.meter_blocked)
    {
      s.append("meter_blocked,");
    }
    if (alarms.back_flow)
    {
      s.append("back_flow,");
    }
    if (alarms.underflow)
    {
      s.append("underflow,");
    }
    if (alarms.overflow)
    {
      s.append("overflow,");
    }
    if (alarms.submarine)
    {
      s.append("submarine,");
    }
    if (alarms.sensor_fraud_currently)
    {
      s.append("sensor_fraud,");
    }
    if (alarms.mechanical_fraud_currently)
    {
      s.append("mechanical_fraud,");
    }
    if (s.length() > 0)
    {
      if (alarms.general_alarm)
      {
        return "general_alarm";
      }
      s.pop_back();
      return s;
    }
    return "no_alarm";
  }

  std::string Driver::previousAlarmsText(IzarRcAlarms &alarms)
  {
    std::string s;
    if (alarms.leakage_previously)
    {
      s.append("leakage,");
    }
    if (alarms.sensor_fraud_previously)
    {
      s.append("sensor_fraud,");
    }
    if (alarms.mechanical_fraud_previously)
    {
      s.append("mechanical_fraud,");
    }
    if (s.length() > 0)
    {
      s.pop_back();
      return s;
    }
    return "no_alarm";
  }

  bool Driver::handleTelegram(AboutTelegram &about, std::vector<uchar> input_frame,
                              bool simulated, std::vector<Address> *addresses,
                              bool *id_match, Telegram *out_analyzed)
  {
    // Log the configured meter_id (from YAML: wmbus_meter.meter_id) if available.
    // The meter_id configured in YAML is fed into MeterInfo::parse(..., aes="id + "," ...)
    // and ends up embedded in the configured address expression id.
    std::vector<AddressExpression> aexps = this->addressExpressions();
    std::string yaml_meter_id = aexps.size() > 0 ? aexps[0].id : "0";
    yaml_meter_id = std::to_string(std::stoul(yaml_meter_id, nullptr, 16));
    ESP_LOGI("APP", "(izar_rc) configured meter_id(yaml)=%s", yaml_meter_id.c_str());


    std::string packet_meter_id;

    const std::vector<uchar> &frame = input_frame;
    if (frame.size() > 10)
    {
      // Bytes frame[4..7] correspond to meter id bytes (endianness adjusted for addressExpressions matching).
      uchar id0 = frame[6], id1 = frame[7], id2 = frame[8], id3 = frame[9];
      packet_meter_id = tostrprintf("%02X%02X%02X%02X", id3, id2, id1, id0);
    }


    ESP_LOGI("APP", "(izar_rc) extracted meter_id(packet)=%s", packet_meter_id.c_str());


    if (packet_meter_id == yaml_meter_id)
    {
        ESP_LOGI("APP", "All the same");
        if (id_match) {
            *id_match = true;
        }
    }

    ESP_LOGI("APP", "(izar_rc) Driver::handleTelegram ENTERED id_match_ptr=%p out_analyzed=%p simulated=%d in_frame_size=%d",
             (void *)id_match, (void *)out_analyzed, (int)simulated, (int)input_frame.size());


    ESP_LOGI("APP",
             "(izar_rc) id_match=%s addresses_ptr=%p addresses_size=%d",
             (id_match && *id_match) ? "true" : "false",
             (void *)addresses,
             addresses ? (int)addresses->size() : -1);


             
    // Cache raw bytes so processContent() can decode.
    cached_input_frame_ = input_frame;

    bool parent_handled = MeterCommonImplementation::handleTelegram(
        about, input_frame, simulated, addresses, id_match, out_analyzed);

    ESP_LOGI("APP", "(izar_rc) handleTelegram parent_handled=%d out_analyzed=%p id_match_post_parent=%s",
             (int)parent_handled, (void *)out_analyzed,
             (id_match && *id_match) ? "true" : "false");


    // If the standardized pipeline did not call processContent(), do it here.
    // This guarantees total/target extraction for this driver.
    if (out_analyzed != NULL && !out_analyzed->discard)
    {
      ESP_LOGI("APP", "(izar_rc) do it here ");
      processContent(out_analyzed);
    }

    return true;
  }

  void Driver::processContent(Telegram *t)
  {
    ESP_LOGW("APP", "IZAR_RC_PROCESSCONTENT_ENTERED t=%p", (void *)t);
    debug("(izar_rc) processContent ENTERED! t=%p", t);

    // Also emit via ESP_LOGI (not DEBUG) so it shows up even if debug() is suppressed.
    ESP_LOGI("APP", "(izar_rc) key count=%d", (int)keys.size());

    if (t == NULL)
    {
      ESP_LOGI("APP", "(izar_rc) processContent early-return: NULL telegram");
      return;
    }

    ESP_LOGW("APP", "IZAR_RC_STATS frame=%d original=%d header=%d suffix=%d dv=%d",
             (int)t->frame.size(), (int)t->original.size(), t->header_size, t->suffix_size, (int)t->dv_entries.size());

    debug("(izar_rc) Telegram %s L=%d rssi=%d",
          t->addresses.back().str().c_str(), t->L, t->about.rssi_dbm);

    // Decode from cached bytes from handleTelegram().
    std::vector<uchar> origin = cached_input_frame_;
    std::vector<uchar> frame = cached_input_frame_;

    if (origin.empty())
    {
      ESP_LOGW("APP", "(izar_rc) processContent: cached_input_frame_ is empty, cannot decode");
      return;
    }

    debug("(izar_rc) frame.size=%d origin.size=%d", (int)frame.size(), (int)origin.size());
    debug("(izar_rc) frame head %s",
          frame.size() >= 15 ? bin2hex(frame).substr(0, 30).c_str() : "?");

    debug("(izar_rc) trying %d keys", (int)keys.size());
    debug("(izar_rc) origin(first 16) %s", origin.size() >= 16 ? bin2hex(std::vector<uchar>(origin.begin(), origin.begin() + 16)).c_str() : "?");
    debug("(izar_rc) frame(first 16) %s", frame.size() >= 16 ? bin2hex(std::vector<uchar>(frame.begin(), frame.begin() + 16)).c_str() : "?");
    debug("(izar_rc) origin_len=%d frame_len=%d", (int)origin.size(), (int)frame.size());
    for (size_t k = 0; k < keys.size() && k < 3; k++)
      debug("(izar_rc) key[%d]=%08x", (int)k, keys[k]);

    std::vector<uchar> decoded_content;
    for (auto &key : keys)
    {
      decoded_content = decodePrios(origin, frame, key);
      if (!decoded_content.empty())
        break;
    }

    if (decoded_content.empty())
    {
      ESP_LOGW("APP", "IZAR_RC_DECODE_FAILED decoded_content empty after success");
      return;
    }

    ESP_LOGW("APP", "IZAR_RC_DECODE_SUCCESS decoded_bytes=%d", (int)decoded_content.size());

    debug("(izar_rc) DECODE SUCCESS! %d bytes: %s", (int)decoded_content.size(),
          bin2hex(decoded_content).c_str());

    // Header (best-effort)
    if (detectDiehlFrameInterpretation(frame) == DiehlFrameInterpretation::SAP_PRIOS)
    {
      std::string digits = std::to_string(
          ((origin[7] & 0x03) << 24) | (origin[6] << 16) | (origin[5] << 8) | origin[4]);
      digits = tostrprintf("%08d", atoi(digits.c_str()));
      uint8_t yy = atoi(digits.substr(0, 2).c_str());
      int year = yy > 70 ? 1900 + yy : 2000 + yy;

      setStringValue("manufacture_year", tostrprintf("%d", year));

      uint32_t serial = atoi(digits.substr(2).c_str());
      setStringValue("serial_number", tostrprintf("%06d", serial));

      uchar supplier_code = '@' + (((origin[9] & 0x0F) << 1) | (origin[8] >> 7));
      uchar meter_type = '@' + ((origin[8] & 0x7C) >> 2);
      uchar diameter = '@' + (((origin[8] & 0x03) << 3) | (origin[7] >> 5));

      std::string prefix = tostrprintf("%c%02d%c%c", supplier_code, yy, meter_type, diameter);
      setStringValue("prefix", prefix);
    }

    char digits[9];
    snprintf(digits, sizeof(digits),
             "%02X%02X%02X%02X",
             origin[9],
             origin[8],
             origin[7],
             origin[6]);
    uint32_t serial = atoi(digits);
    setStringValue("serial_number", tostrprintf("%08u", serial));

    // Battery & TX
    double battery_y = (frame[12] & 0x1F) / 2.0;
    setNumericValue("remaining_battery_life", Unit::Year, battery_y);

    int tx_s = 1 << ((frame[11] & 0x0F) + 2);
    setNumericValue("transmit_period", Unit::Second, tx_s);

    // Volumes (payload layout differs per firmware).
    // Diagnostic: dump candidate uint32 values at various offsets.
    debug("(izar_rc) decoded_content hex=%s len=%d",
          bin2hex(decoded_content).c_str(), (int)decoded_content.size());

    auto try_u32_at = [&](size_t off, bool *ok) -> double
    {
      if (off + 4 > decoded_content.size())
      {
        *ok = false;
        return 0;
      }
      *ok = true;
      return uint32FromBytes(decoded_content, off, true);
    };

    for (size_t off = 0; off <= 7; off++)
    {
      bool ok_u32 = false;
      double v = try_u32_at(off, &ok_u32);
      if (ok_u32)
      {
        debug("(izar_rc) u32 candidate off=%d => %u", (int)off, (unsigned)v);
      }
    }

    // Current best-guess offsets (kept as defaults, but will be corrected after we see logs).
    bool ok_total = false;
    bool ok_last = false;
    double total_l = try_u32_at(1, &ok_total);
    double last_month_l = try_u32_at(5, &ok_last);

    double total_m3 = total_l / 1000.0;
    double last_month_m3 = last_month_l / 1000.0;

    setNumericValue("total", Unit::L, total_l);
    setNumericValue("total", Unit::M3, total_m3);

    setNumericValue("target", Unit::L, last_month_l);
    setNumericValue("target", Unit::M3, last_month_m3);

    ESP_LOGI("APP", "(izar_rc) setNumericValue serial=%u total_l=%.3f total_m3=%.6f last_month_l=%.3f last_month_m3=%.6f",
             serial, total_l, total_m3, last_month_l, last_month_m3);

    // Date (best-effort)
    if (decoded_content.size() > 10)
    {
      uint16_t year =
          ((decoded_content[10] & 0xF0) >> 1) + ((decoded_content[9] & 0xE0) >> 5);
      if (year > 80)
        year += 1900;
      else
        year += 2000;

      uint8_t month = decoded_content[10] & 0xF;
      uint8_t day = decoded_content[9] & 0x1F;

      std::string computed_date =
          tostrprintf("%d-%02d-%02d", (int)year, (int)month, (int)day);

      ESP_LOGI("APP",
               "(izar_rc) last_month_measure_date debug decoded_content.size=%d decoded[9]=0x%02X decoded[10]=0x%02X => year=%u month=%u day=%u computed=%s",
               (int)decoded_content.size(), decoded_content[9],
               decoded_content[10], (unsigned)year, (unsigned)month,
               (unsigned)day, computed_date.c_str());

      setStringValue("last_month_measure_date", computed_date);
    }
    else
    {
      ESP_LOGW("APP",
               "(izar_rc) last_month_measure_date debug: decoded_content too small (size=%d)",
               (int)decoded_content.size());
    }

    // Alarms
    IzarRcAlarms alarms;
    alarms.general_alarm = frame[11] >> 7;
    alarms.leakage_currently = frame[12] >> 7;
    alarms.leakage_previously = (frame[12] >> 6) & 1;
    alarms.meter_blocked = (frame[12] >> 5) & 1;
    alarms.back_flow = frame[13] >> 7;
    alarms.underflow = (frame[13] >> 6) & 1;
    alarms.overflow = (frame[13] >> 5) & 1;
    alarms.submarine = (frame[13] >> 4) & 1;
    alarms.sensor_fraud_currently = (frame[13] >> 3) & 1;
    alarms.sensor_fraud_previously = (frame[13] >> 2) & 1;
    alarms.mechanical_fraud_currently = (frame[13] >> 1) & 1;
    alarms.mechanical_fraud_previously = frame[13] & 1;

    setStringValue("current_alarms", currentAlarmsText(alarms));
    setStringValue("previous_alarms", previousAlarmsText(alarms));

    ESP_LOGI("APP", "(izar_rc) total_m3=%.3f last_month_total_m3=%.3f alarms=%s battery=%.1fy tx=%ds",
             total_m3, last_month_m3, currentAlarmsText(alarms).c_str(), battery_y, tx_s);
  }

  std::vector<uchar> Driver::decodePrios(const std::vector<uchar> &origin,
                                         const std::vector<uchar> &frame,
                                         uint32_t key)
  {
    return decodeDiehlLfsr(origin, frame, key, DiehlLfsrCheckMethod::HEADER_1_BYTE, 0x4B);
  }

  bool Driver::extractIzarRcId(const std::vector<uchar> &frame, std::string *id_str)
  {
    if (frame.size() < 10)
      return false;

    // Bytes frame[4..7] correspond to meter id bytes (endianness adjusted for addressExpressions matching).
    uchar id0 = frame[4], id1 = frame[5], id2 = frame[6], id3 = frame[7];
    *id_str = tostrprintf("%02X%02X%02X%02X", id3, id2, id1, id0);
    return true;
  }

} // namespace
