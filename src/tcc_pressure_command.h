#ifndef TCC_PRESSURE_COMMAND_H
#define TCC_PRESSURE_COMMAND_H

// Keep the actuator-side correction available to host tests. The production
// path historically truncates the corrected float to an integer command.
inline int tcc_final_pressure_command_mbar(
    int controller_pressure_mbar,
    float temperature_multiplier
) {
    const int corrected = static_cast<int>(
        static_cast<float>(controller_pressure_mbar) * temperature_multiplier
    );
    return corrected < 0 ? 0 : (corrected > 3000 ? 3000 : corrected);
}

#endif
