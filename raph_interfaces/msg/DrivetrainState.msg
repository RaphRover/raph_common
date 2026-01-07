# Represents the state of the drivetrain

# Current steering mode.
raph_interfaces/SteeringMode steering_mode

uint8 OPERATING_STATE_DISABLED=0
uint8 OPERATING_STATE_ENABLED=1
uint8 OPERATING_STATE_CALIBRATING_SERVOS=2
uint8 OPERATING_STATE_CHANGING_STEERING_MODE=3

# Current operating state.
uint8 operating_state

# Indicates if the servos are calibrated.
bool is_servos_calibrated