#include "battery_soc.h"

namespace {
struct SocPoint {
    int millivolts;
    int percent;
};

// 这些曲线是常温、低倍率、近似静置电压的首版参考表。
// 带载压降、温度和电芯老化都会造成偏差，因此界面明确标注为 SOC 参考值。
constexpr SocPoint LI_ION_CURVE[] = {
    {3000, 0}, {3300, 5}, {3500, 10}, {3600, 20}, {3700, 35},
    {3750, 50}, {3800, 60}, {3900, 75}, {4000, 85}, {4100, 95},
    {4200, 100},
};

constexpr SocPoint LIFEPO4_CURVE[] = {
    {2500, 0}, {2900, 2}, {3000, 5}, {3100, 10}, {3200, 18},
    {3250, 28}, {3300, 45}, {3320, 55}, {3340, 65}, {3380, 78},
    {3450, 90}, {3550, 97}, {3650, 100},
};

constexpr SocPoint LIPO_HV_CURVE[] = {
    {3000, 0}, {3400, 5}, {3600, 10}, {3700, 20}, {3800, 35},
    {3900, 50}, {4000, 65}, {4100, 78}, {4200, 88}, {4300, 96},
    {4350, 100},
};

constexpr SocPoint LEAD_ACID_CURVE[] = {
    {1750, 0}, {1850, 5}, {1900, 10}, {1980, 25}, {2050, 50},
    {2100, 75}, {2120, 90}, {2150, 100},
};

template <size_t N>
BatterySocEstimate interpolate_curve(const SocPoint (&curve)[N],
                                     int pack_voltage_mv, int cells) {
    BatterySocEstimate result;
    cells = max(cells, 1);
    result.per_cell_mv = static_cast<int>(
        (static_cast<int64_t>(pack_voltage_mv) + cells / 2) / cells);

    if (result.per_cell_mv <= curve[0].millivolts) {
        result.percent = curve[0].percent;
        result.below_curve = result.per_cell_mv < curve[0].millivolts;
        return result;
    }
    if (result.per_cell_mv >= curve[N - 1].millivolts) {
        result.percent = curve[N - 1].percent;
        result.above_curve = result.per_cell_mv > curve[N - 1].millivolts;
        return result;
    }

    for (size_t i = 1; i < N; ++i) {
        if (result.per_cell_mv > curve[i].millivolts) {
            continue;
        }
        const SocPoint &low = curve[i - 1];
        const SocPoint &high = curve[i];
        const int voltage_span = high.millivolts - low.millivolts;
        const int percent_span = high.percent - low.percent;
        const int offset = result.per_cell_mv - low.millivolts;
        result.percent = low.percent + static_cast<int>(
            (static_cast<int64_t>(offset) * percent_span + voltage_span / 2) /
            voltage_span);
        result.percent = constrain(result.percent, 0, 100);
        return result;
    }

    result.percent = 0;
    return result;
}
}  // namespace

const char *battery_chemistry_name(BatteryChemistry chemistry) {
    switch (chemistry) {
        case BatteryChemistry::LiIon: return "三元锂/锂离子";
        case BatteryChemistry::LiFePO4: return "磷酸铁锂";
        case BatteryChemistry::LiPoHv: return "高压锂聚合物";
        case BatteryChemistry::LeadAcid: return "铅酸";
        default: return "未知";
    }
}

const char *battery_chemistry_short_name(BatteryChemistry chemistry) {
    switch (chemistry) {
        case BatteryChemistry::LiIon: return "三元锂";
        case BatteryChemistry::LiFePO4: return "磷铁锂";
        case BatteryChemistry::LiPoHv: return "高压锂";
        case BatteryChemistry::LeadAcid: return "铅酸";
        default: return "未知";
    }
}

int battery_chemistry_min_mv(BatteryChemistry chemistry) {
    switch (chemistry) {
        case BatteryChemistry::LiIon: return 3000;
        case BatteryChemistry::LiFePO4: return 2500;
        case BatteryChemistry::LiPoHv: return 3000;
        case BatteryChemistry::LeadAcid: return 1750;
        default: return 3000;
    }
}

int battery_chemistry_max_mv(BatteryChemistry chemistry) {
    switch (chemistry) {
        case BatteryChemistry::LiIon: return 4200;
        case BatteryChemistry::LiFePO4: return 3650;
        case BatteryChemistry::LiPoHv: return 4350;
        case BatteryChemistry::LeadAcid: return 2150;
        default: return 4200;
    }
}

BatterySocEstimate estimate_battery_soc(BatteryChemistry chemistry,
                                        int pack_voltage_mv,
                                        int cells) {
    switch (chemistry) {
        case BatteryChemistry::LiIon:
            return interpolate_curve(LI_ION_CURVE, pack_voltage_mv, cells);
        case BatteryChemistry::LiFePO4:
            return interpolate_curve(LIFEPO4_CURVE, pack_voltage_mv, cells);
        case BatteryChemistry::LiPoHv:
            return interpolate_curve(LIPO_HV_CURVE, pack_voltage_mv, cells);
        case BatteryChemistry::LeadAcid:
            return interpolate_curve(LEAD_ACID_CURVE, pack_voltage_mv, cells);
        default:
            return interpolate_curve(LI_ION_CURVE, pack_voltage_mv, cells);
    }
}
