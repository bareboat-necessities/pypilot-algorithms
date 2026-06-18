#pragma once

#include <stdint.h>
#include <math.h>
#include <pypilot_syslib.hpp>

extern "C" {
#include "../../extras/wmm_tinier/wmm.h"
}

namespace pypilot_algorithms {
namespace wmm {

struct Date {
    int year;
    int month;
    int day;

    Date() : year(2025), month(1), day(1) {}
    Date(int y, int m, int d) : year(y), month(m), day(d) {}
};

struct Position {
    float latitude_deg;
    float longitude_deg;
    float altitude_m;

    Position() : latitude_deg(0.0f), longitude_deg(0.0f), altitude_m(0.0f) {}
    Position(float lat, float lon, float alt = 0.0f)
        : latitude_deg(lat), longitude_deg(lon), altitude_m(alt) {}
};

struct MagneticResult {
    bool valid;
    float declination_deg;

    MagneticResult() : valid(false), declination_deg(0.0f) {}
};

inline bool is_leap_year(int year) {
    return ((year % 4) == 0 && (year % 100) != 0) || ((year % 400) == 0);
}

inline int days_in_month(int year, int month) {
    static const int days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if (month < 1 || month > 12) return 0;
    if (month == 2 && is_leap_year(year)) return 29;
    return days[month - 1];
}

inline bool valid_date(const Date& date) {
    if (date.year < 2025 || date.year > 2030) return false;
    const int dim = days_in_month(date.year, date.month);
    return dim > 0 && date.day >= 1 && date.day <= dim;
}

inline bool valid_position(const Position& position) {
    return position.latitude_deg >= -90.0f && position.latitude_deg <= 90.0f;
}

inline float normalize_longitude(float lon) {
    while (lon > 180.0f) lon -= 360.0f;
    while (lon < -180.0f) lon += 360.0f;
    return lon;
}

class WorldMagneticModel {
public:
    WorldMagneticModel() : initialized_(false), logger_(0) {}

    void set_logger(pypilot_syslib::Logger* logger) { logger_ = logger; }
    pypilot_syslib::Logger* logger() const { return logger_; }

    bool begin() {
        if (!initialized_) {
            wmm_init();
            initialized_ = true;
        }
        return true;
    }

    MagneticResult evaluate(const Position& position, const Date& date) {
        MagneticResult result;
        if (!valid_position(position) || !valid_date(date)) {
            log_invalid(0);
            return result;
        }

        begin();
        const uint8_t yy = uint8_t(date.year - 2000);
        const float model_date = wmm_get_date(yy, uint8_t(date.month), uint8_t(date.day));
        float declination = 0.0f;
        E0000(position.latitude_deg, normalize_longitude(position.longitude_deg), model_date, &declination);
        result.valid = isfinite(declination) != 0;
        result.declination_deg = declination;
        if (!result.valid) log_invalid(1);
        return result;
    }

    bool declination_deg(float latitude_deg,
                         float longitude_deg,
                         int year,
                         int month,
                         int day,
                         float& out_deg) {
        const MagneticResult result = evaluate(Position(latitude_deg, longitude_deg), Date(year, month, day));
        if (!result.valid) return false;
        out_deg = result.declination_deg;
        return true;
    }

private:
    void log_invalid(int32_t code) const {
        pypilot_syslib::log_if(logger_, 0ULL,
                               pypilot_syslib::LogLevel::Warn,
                               pypilot_syslib::LogModule::Algorithms,
                               pypilot_syslib::LogEvent::WmmEvaluationInvalid,
                               "wmm evaluation invalid",
                               code);
    }

    bool initialized_;
    pypilot_syslib::Logger* logger_;
};

inline bool declination_deg(float latitude_deg,
                            float longitude_deg,
                            int year,
                            int month,
                            int day,
                            float& out_deg) {
    static WorldMagneticModel model;
    return model.declination_deg(latitude_deg, longitude_deg, year, month, day, out_deg);
}

} // namespace wmm
} // namespace pypilot_algorithms
