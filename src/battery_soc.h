#pragma once

#include <Arduino.h>

enum class BatteryChemistry : uint8_t {
    LiIon = 0,
    LiFePO4,
    LiPoHv,
    LeadAcid,
    Count,
};

struct BatterySocEstimate {
    int percent = 0;
    int per_cell_mv = 0;
    bool below_curve = false;
    bool above_curve = false;
};

const char *battery_chemistry_name(BatteryChemistry chemistry);
const char *battery_chemistry_short_name(BatteryChemistry chemistry);
int battery_chemistry_min_mv(BatteryChemistry chemistry);
int battery_chemistry_max_mv(BatteryChemistry chemistry);
BatterySocEstimate estimate_battery_soc(BatteryChemistry chemistry,
                                        int pack_voltage_mv,
                                        int cells);
