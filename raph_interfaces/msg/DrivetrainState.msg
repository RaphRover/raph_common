# Represents the state of the drivetrain

# Current steering mode.
raph_interfaces/SteeringMode steering_mode

uint8 DISABLED=0
uint8 ENABLED=1
uint8 CALIBRATING_SERVOS=2
uint8 CHANGING_STEERING_MODE=3

# Current operating state.
uint8 operating_state

# Indicates if the servos are calibrated.
bool is_servos_calibrated