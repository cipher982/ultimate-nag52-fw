#include "tcc_transient_controller.h"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>

namespace {

const char* state_name(TccTransientState state) {
    switch (state) {
    case TccTransientState::Open: return "open";
    case TccTransientState::Fill: return "fill";
    case TccTransientState::SlipControl: return "slip_control";
    case TccTransientState::Locked: return "locked";
    case TccTransientState::ReleaseFault: return "release_fault";
    }
    return "unknown";
}

const char* reason_name(TccTransientReason reason) {
    switch (reason) {
    case TccTransientReason::None: return "none";
    case TccTransientReason::DemandOpen: return "demand_open";
    case TccTransientReason::ShiftInhibit: return "shift_inhibit";
    case TccTransientReason::GearMismatch: return "gear_mismatch";
    case TccTransientReason::PostShiftSettling: return "post_shift_settling";
    case TccTransientReason::InvalidSpeed: return "invalid_speed";
    case TccTransientReason::TargetHysteresis: return "target_hysteresis";
    case TccTransientReason::InvalidPressure: return "invalid_pressure";
    case TccTransientReason::ExcessiveSlipRate: return "excessive_slip_rate";
    case TccTransientReason::ContactNotDetected: return "contact_not_detected";
    case TccTransientReason::CoastOverrun: return "coast_overrun";
    }
    return "unknown";
}

bool parse_bool(int value, const char* name, std::size_t line_number) {
    if (value != 0 && value != 1) {
        throw std::runtime_error("line " + std::to_string(line_number) +
            ": " + name + " must be 0 or 1");
    }
    return value != 0;
}

void replay(std::istream& input) {
    std::cout << "index now_ms state reason pressure feedforward feedback integral "
                 "rate_relief slip delta target trajectory contact\n";

    std::string line;
    std::size_t line_number = 0;
    std::size_t index = 0;
    while (std::getline(input, line)) {
        ++line_number;
        for (char& character : line) {
            if (character == ',' || character == '\t') character = ' ';
        }
        std::istringstream row(line);
        std::string first;
        if (!(row >> first)) continue;
        if (first[0] == '#') continue;
        if (first == "now_ms" || first == "time_ms") continue;

        uint32_t now_ms = 0;
        int request_apply = 0;
        int force_open = 0;
        int speed_valid = 0;
        int signed_slip_rpm = 0;
        int target_slip_rpm = 0;
        int feedforward_pressure = 0;
        int gear = 0;
        int coast_mode = 0;
        try {
            std::istringstream first_value(first);
            if (!(first_value >> now_ms) || !first_value.eof() ||
                !(row >> request_apply >> force_open >> speed_valid >> signed_slip_rpm >>
                    target_slip_rpm >> feedforward_pressure >> gear >> coast_mode)) {
                throw std::runtime_error("expected 9 columns: now_ms request_apply "
                    "force_open speed_valid signed_slip_rpm target_slip_rpm "
                    "feedforward_pressure gear coast_mode");
            }
        } catch (const std::exception& error) {
            throw std::runtime_error("line " + std::to_string(line_number) +
                ": " + error.what());
        }
        if (gear < 0 || gear > 255) {
            throw std::runtime_error("line " + std::to_string(line_number) +
                ": gear must be between 0 and 255");
        }
        const bool apply = parse_bool(request_apply, "request_apply", line_number);
        const bool open = parse_bool(force_open, "force_open", line_number);
        const bool valid = parse_bool(speed_valid, "speed_valid", line_number);
        const bool coast = parse_bool(coast_mode, "coast_mode", line_number);

        static TccTransientController controller;
        controller.select_gear(static_cast<uint8_t>(gear));
        const auto output = controller.step({
            apply,
            open,
            valid,
            signed_slip_rpm,
            target_slip_rpm,
            feedforward_pressure,
            now_ms,
            coast,
        });
        std::cout << index++ << ' ' << now_ms << ' ' << state_name(output.state) << ' '
                  << reason_name(output.reason) << ' ' << output.pressure << ' '
                  << output.feedforward_pressure << ' ' << output.feedback_correction << ' '
                  << output.integral_correction << ' ' << output.slip_rate_correction << ' '
                  << output.slip_rpm << ' ' << output.slip_delta_rpm << ' '
                  << output.target_slip_rpm << ' ' << output.trajectory_slip_rpm << ' '
                  << (output.contact_detected ? 1 : 0) << '\n';
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 2 || (argc == 2 && std::string(argv[1]) == "--help")) {
        std::cerr << "usage: host_tcc_replay [input-file]\n"
                     "input columns: now_ms request_apply force_open speed_valid "
                     "signed_slip_rpm target_slip_rpm feedforward_pressure gear coast_mode\n"
                     "use '-' or no input-file for stdin\n";
        return argc > 2 ? 2 : 0;
    }

    try {
        if (argc == 2 && std::string(argv[1]) != "-") {
            std::ifstream input(argv[1]);
            if (!input) {
                std::cerr << "cannot open replay input: " << argv[1] << '\n';
                return 2;
            }
            replay(input);
        } else {
            replay(std::cin);
        }
    } catch (const std::exception& error) {
        std::cerr << "host_tcc_replay: " << error.what() << '\n';
        return 2;
    }
    return 0;
}
