# This message represents the state of a single LED.

# The duration (in ms) the state should be active.
# Setting it to 0 will make the state active indefinitely.
uint16 duration

# The priority of the state.
# If the value is equal to 0 or is lower than the priority of the current user LED, the state is ignored.
# The value of -1 resets the user state of the LED.
int8 priority

# The color of the LED
raph_interfaces/LedColor color
