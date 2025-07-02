# Diagnostic information about motors controlled by RaphCore.
#
# The order of the motors is:
#   - rear left wheel
#   - rear right wheel
#   - right servo
#   - left servo
#   - front left wheel
#   - front right wheel

std_msgs/Header header

# The current temperature in degrees Celsius.
uint8[6] temperature

# The current power output in watts.
float64[6] power

# The energy consumed in watt-hours since the robot was powered on.
float64[6] energy

uint8 FAULT_BIT_SENSOR_FAULT=1
uint8 FAULT_BIT_BUS_OVERCURRENT=2
uint8 FAULT_BIT_PHASE_OVERCURRENT=4
uint8 FAULT_BIT_STALL=8
uint8 FAULT_BIT_OVERHEAT=16

# The current fault mask reported by the motor.
uint8[6] fault_mask_active

# Mask of all fault that occurred since the robot was powered on.
uint8[6] fault_mask_occurred
