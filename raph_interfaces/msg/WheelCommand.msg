# This message defines an interface of steering a single wheel

# Defines whether the motor is enabled or disabled (loose)
bool enabled

# Target velocity of the wheel in rad/s.
float64 target_velocity

# The divider for the maximum acceleration of the wheels (10000 rpm/s^2)
# E.g. 1 = 10000 rpm/s^2, 2 = 5000 rpm/s^2, 4 = 2500 rpm/s^2 etc.
# If set to 0 will use the value of drivetrain.wheel.acceleration_divider parameter
uint8 acceleration_divider