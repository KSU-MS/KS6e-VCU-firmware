#ifndef DISTANCE_TRACKER_H
#define DISTANCE_TRACKER_H
#include <stdint.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
struct old_new_t
{
    float newval = 0;
    float oldval = 0;
};

struct energy_data_t
{
    int16_t energy_wh;
    int16_t eff_inst;
    int16_t distance_m;
    int16_t efficiency_kmkwh;
    energy_data_t(int16_t wh, int16_t eff, int16_t m, int16_t kmkwh) : energy_wh(wh), eff_inst(eff), distance_m(m), efficiency_kmkwh(kmkwh) {}
};

class distance_tracker_s
{
public:
    void tick(unsigned long newtime)
    {
        time = newtime;
    }
    void update(float amps, float volts, float rpm, float circumference, unsigned long newtime)
    {
        // calc elapsed time
        unsigned long elapsed_time_us = newtime - this->time;
        float elapsed_time_seconds = static_cast<float>(elapsed_time_us) / 1000000;
        // update time
        this->time = newtime;
        // calculate distance travelled during the last update time
        distance_km += elapsed_time_seconds * velocity_ms.oldval;
        Serial.println(velocity_ms.oldval);
        // calculate power used during the last update time
        energy_kwh += elapsed_time_seconds / 3600 * power_kw.oldval;

        // calculate amp hrs used during the last update time
        capacity_ah += elapsed_time_seconds / 3600 * current_amps.oldval;

        // update efficiencies
        efficiency_kmkwh = distance_km/energy_kwh;
        efficiency_instantaneous = (elapsed_time_seconds * velocity_ms.oldval) / (elapsed_time_seconds / 3600 * power_kw.oldval);
        // set old vals to the previous new one
        power_kw.oldval = power_kw.newval;
        current_amps.oldval = current_amps.newval;
        velocity_ms.oldval = velocity_ms.newval;

        // set new vals to the newest
        current_amps.newval = amps;
        // Calculate power
        power_kw.newval = amps * volts;
        // Calculate speed
        velocity_ms.newval = rpm * circumference;
    }
    energy_data_t get_data()
    {
        energy_data_t data = energy_data_t(static_cast<int16_t>(energy_kwh*1000), static_cast<int16_t>(efficiency_instantaneous*1000), static_cast<int16_t>(distance_km*1000), static_cast<int16_t>(efficiency_kmkwh*1000));
        return data;
    }

private:
    old_new_t power_kw;
    old_new_t current_amps;
    old_new_t velocity_ms;
    unsigned long time;
    float capacity_ah;
    float energy_kwh = 0;
    float distance_km = 0;
    float efficiency_kmkwh = 0;
    float efficiency_instantaneous = 0;
};

#endif