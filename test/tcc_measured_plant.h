#ifndef TEST_TCC_MEASURED_PLANT_H
#define TEST_TCC_MEASURED_PLANT_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>

namespace tcc_test {

// Sanitized A0/A1 readback from the vehicle before the V8 drive. The host
// plant uses the learned cells rather than the generic firmware defaults.
struct MeasuredTccMaps {
    static constexpr std::array<int, 13> kLoadPercent = {
        0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 125, 150,
    };

    static constexpr std::array<int, 65> kSlipMbar = {
        581, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400,
        1004, 1104, 1104, 1133, 1970, 2100, 2390, 2600, 2800, 3037, 3200, 3400, 3600,
        1062, 1132, 1182, 1195, 1650, 2320, 2420, 2780, 2990, 3160, 3210, 3600, 3800,
        233, 719, 1162, 1192, 1192, 1192, 2660, 3000, 3200, 3400, 3600, 3800, 4000,
        428, 1242, 1242, 1242, 1242, 1242, 1327, 2930, 2930, 3480, 3770, 4000, 4200,
    };

    static constexpr std::array<int, 65> kLockMbar = {
        1000, 2000, 2415, 2600, 2800, 3000, 3250, 3500, 3750, 4000, 4500, 5000, 6000,
        1417, 2390, 2600, 2792, 2994, 3196, 3426, 4000, 4500, 5000, 5500, 6000, 7000,
        1169, 2558, 2745, 2765, 2784, 3082, 3597, 3994, 4500, 5000, 5500, 6000, 7000,
        1124, 2800, 2986, 3049, 3271, 3600, 3830, 5000, 5500, 6000, 6500, 7000, 8000,
        1011, 1198, 1198, 1198, 1198, 1198, 1198, 5014, 5500, 6000, 6500, 7000, 8000,
    };

    static double lookup(const std::array<int, 65>& map, int gear, double load_percent) {
        const int bounded_gear = std::max(1, std::min(5, gear));
        const std::size_t row = static_cast<std::size_t>(bounded_gear - 1) * kLoadPercent.size();
        if (load_percent <= kLoadPercent.front()) {
            return map[row];
        }
        if (load_percent >= kLoadPercent.back()) {
            return map[row + kLoadPercent.size() - 1];
        }
        for (std::size_t i = 1; i < kLoadPercent.size(); ++i) {
            if (load_percent <= kLoadPercent[i]) {
                const double span = kLoadPercent[i] - kLoadPercent[i - 1];
                const double fraction = (load_percent - kLoadPercent[i - 1]) / span;
                return map[row + i - 1] + fraction * (map[row + i] - map[row + i - 1]);
            }
        }
        return map[row + kLoadPercent.size() - 1];
    }

    static double slip_mbar(int gear, double load_percent) {
        return lookup(kSlipMbar, gear, load_percent);
    }

    static double lock_mbar(int gear, double load_percent) {
        return lookup(kLockMbar, gear, load_percent);
    }
};

struct MeasuredPlantCalibration {
    // Median post-controller multiplier during the stable D4 event. Individual
    // loaded 2->3 observations span 0.864-0.914.
    double temperature_multiplier = 0.90;

    // Effective-pressure model fitted to the observed D4 release/re-grab
    // envelope: final command about 1080-1208 mbar at release and 1252-1326
    // mbar at re-grab around 150 Nm.
    double kiss_pressure_mbar = 700.0;
    double static_capacity_nm_per_mbar = 0.375;
    double kinetic_capacity_nm_per_mbar = 0.300;

    // A reduced relative-inertia model. It is not a line-pressure estimator;
    // it only reproduces the measured slip response to command changes.
    double relative_accel_rpm_per_second_per_nm = 100.0;
    double viscous_drag_nm_per_rpm = 0.10;

    // Three 20 ms cycles of command dead time plus asymmetric first-order
    // hydraulic response. Prior art requires delay/fill dynamics; these values
    // are bounded by the measured 110-182 ms breakaway-to-peak and 246-304 ms
    // breakaway-to-re-grab intervals.
    std::size_t command_delay_cycles = 3;
    double pressure_rise_tau_ms = 110.0;
    double pressure_fall_tau_ms = 140.0;
    double regrab_slip_rpm = 18.0;
    double regrab_capacity_margin_nm = 8.0;
};

struct MeasuredPlantOutput {
    double slip_rpm = 0.0;
    double final_command_mbar = 0.0;
    double effective_pressure_mbar = 0.0;
    double static_capacity_nm = 0.0;
    double kinetic_capacity_nm = 0.0;
    bool coupled = false;
};

// Hybrid wet-clutch plant: delayed hydraulics + separate static/kinetic
// capacity + a stick/slip transition. A smooth linear plant cannot reproduce
// the measured limit cycle because it has no breakaway boundary.
class MeasuredWetTccPlant {
public:
    explicit MeasuredWetTccPlant(MeasuredPlantCalibration calibration = {})
        : calibration_(calibration) {
        reset_open();
    }

    void reset_open(double initial_slip_rpm = 300.0) {
        delayed_commands_.assign(calibration_.command_delay_cycles + 1, 0.0);
        effective_pressure_mbar_ = 0.0;
        slip_rpm_ = std::max(0.0, initial_slip_rpm);
        coupled_ = false;
    }

    void prime_coupled(double controller_pressure_mbar) {
        const double final_command = controller_pressure_mbar * calibration_.temperature_multiplier;
        delayed_commands_.assign(calibration_.command_delay_cycles + 1, final_command);
        effective_pressure_mbar_ = final_command;
        slip_rpm_ = 0.0;
        coupled_ = true;
    }

    MeasuredPlantOutput step(
        double controller_pressure_mbar,
        double input_torque_nm,
        double dt_ms = 20.0
    ) {
        const double torque_magnitude_nm = std::abs(input_torque_nm);
        const double final_command_mbar = std::max(
            0.0,
            controller_pressure_mbar * calibration_.temperature_multiplier
        );
        delayed_commands_.push_back(final_command_mbar);
        const double delayed_command_mbar = delayed_commands_.front();
        delayed_commands_.pop_front();

        const double tau_ms = delayed_command_mbar >= effective_pressure_mbar_
            ? calibration_.pressure_rise_tau_ms
            : calibration_.pressure_fall_tau_ms;
        const double alpha = std::min(1.0, dt_ms / tau_ms);
        effective_pressure_mbar_ += alpha * (delayed_command_mbar - effective_pressure_mbar_);

        const double pressure_above_kiss = std::max(
            0.0,
            effective_pressure_mbar_ - calibration_.kiss_pressure_mbar
        );
        const double static_capacity_nm =
            pressure_above_kiss * calibration_.static_capacity_nm_per_mbar;
        const double kinetic_capacity_nm =
            pressure_above_kiss * calibration_.kinetic_capacity_nm_per_mbar;

        if (coupled_ && torque_magnitude_nm > static_capacity_nm) {
            coupled_ = false;
            slip_rpm_ = 1.0;
        }

        if (!coupled_) {
            const double drag_nm = calibration_.viscous_drag_nm_per_rpm * slip_rpm_;
            const double accelerating_torque_nm =
                torque_magnitude_nm - kinetic_capacity_nm - drag_nm;
            slip_rpm_ += accelerating_torque_nm *
                calibration_.relative_accel_rpm_per_second_per_nm *
                (dt_ms / 1000.0);
            slip_rpm_ = std::max(0.0, slip_rpm_);

            if (slip_rpm_ <= calibration_.regrab_slip_rpm &&
                kinetic_capacity_nm >=
                    torque_magnitude_nm + calibration_.regrab_capacity_margin_nm) {
                coupled_ = true;
                slip_rpm_ = 0.0;
            }
        } else {
            slip_rpm_ = 0.0;
        }

        return {
            slip_rpm_,
            final_command_mbar,
            effective_pressure_mbar_,
            static_capacity_nm,
            kinetic_capacity_nm,
            coupled_,
        };
    }

    bool coupled() const { return coupled_; }
    double slip_rpm() const { return slip_rpm_; }
    double effective_pressure_mbar() const { return effective_pressure_mbar_; }

private:
    MeasuredPlantCalibration calibration_;
    std::deque<double> delayed_commands_;
    double effective_pressure_mbar_ = 0.0;
    double slip_rpm_ = 0.0;
    bool coupled_ = false;
};

}  // namespace tcc_test

#endif
