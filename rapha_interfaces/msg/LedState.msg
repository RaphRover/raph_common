# This message represents the state of a single LED

# The duration (in ms) the state should be active
uint16 duration

# The priority of the state. The state with the highest priority is displayed
# The value of -1 resets the user state of the LED
int8 priority

# The color of the LED
rapha_interfaces/LedColor color
