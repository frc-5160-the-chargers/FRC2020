motor_ports: {
    drivetrain: {
        left: [5, 6, 7]
        right: [2, 3, 4]

        left_encoder: 5
        right_encoder: 2
    }
}

constants: {
    ticks_per_rotation: 4096
    wheel_diameter: 6
}

motor_config: {
    drivetrain: {
        voltage_saturation: 11
        deadband: 0.025

        peak_current: 100
        continuous_current: 40

        deafult_mode: "break"
    }
}

subsystem_config: {
    drivetrain: {
        rotation_deadzone: 0.1
    }
    navx: {
        samples_taking: 5
    }
}

subsystems_enabled: [
    "drivetrain"
]