# This message represents the state of a single LED

# The duration (in ms) the state should be active
uint16 duration

# The priority of the state. The state with the highest priority is displayed
uint8 priority

# The color of the LED
leo2_msgs/LedColor color
