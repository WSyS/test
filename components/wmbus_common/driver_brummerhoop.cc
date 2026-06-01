// Simplified Brummerhoop driver
// Fallback-only implementation

#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"

struct Driver : public virtual MeterCommonImplementation
{
    Driver(MeterInfo &mi, DriverInfo &di) :
        MeterCommonImplementation(mi, di)
    {
        di.usesProcessContent();
    }

    bool handleTelegram(AboutTelegram &about,
                        std::vector<uchar> input_frame,
                        bool simulated,
                        std::vector<Address> *addresses,
                        bool *id_match,
                        Telegram *out_analyzed = nullptr) override
    {
        last_frame_ = input_frame;

        if (out_analyzed != nullptr && !out_analyzed->discard)
        {
            processContent(out_analyzed);
        }

        return true;
    }

    void processContent(Telegram *t) override
    {
        const auto &frame = last_frame_;

        // Preferred Brummerhoop pattern:
        // 04 6D <4-byte date> 04 13 <4-byte volume>
        for (size_t i = 0; i + 12 <= frame.size(); i++)
        {
            if (frame[i]     == 0x04 &&
                frame[i + 1] == 0x6D &&
                frame[i + 6] == 0x04 &&
                frame[i + 7] == 0x13)
            {
                uint32_t raw =
                    uint32_t(frame[i + 8]) |
                    (uint32_t(frame[i + 9]) << 8) |
                    (uint32_t(frame[i + 10]) << 16) |
                    (uint32_t(frame[i + 11]) << 24);

                setNumericValue("total", Unit::M3, raw / 1000.0);
                return;
            }
        }

        // Generic fallback
        for (size_t i = 0; i + 6 <= frame.size(); i++)
        {
            if (frame[i] == 0x04 &&
                frame[i + 1] == 0x13)
            {
                uint32_t raw =
                    uint32_t(frame[i + 2]) |
                    (uint32_t(frame[i + 3]) << 8) |
                    (uint32_t(frame[i + 4]) << 16) |
                    (uint32_t(frame[i + 5]) << 24);

                setNumericValue("total", Unit::M3, raw / 1000.0);
                return;
            }
        }
    }

private:
    std::vector<uchar> last_frame_;
};
