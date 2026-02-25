# This message defines an interface of steering a single servo

# Defines whether the motor is enabled or disabled (loose)
bool enabled

# Servo target position in rad
float64 target_position

# Target velocity of the servo in rad/s.
# If set to 0 will use the value of drivetrain.servo.position_loop_speed parameter
float64 target_velocity

# The divider for the maximum acceleration of the servos (10000 rpm/s^2)
# E.g. 1 = 10000 rpm/s^2, 2 = 5000 rpm/s^2, 4 = 2500 rpm/s^2 etc.
# If set to 0 will use the value of drivetrain.servo.acceleration_divider parameter
uint8 acceleration_divider