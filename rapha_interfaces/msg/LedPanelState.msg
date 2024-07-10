# Represents the state of the whole LED panel.

# Which LED panel does this state applies to.
rapha_interfaces/LedPanel panel

# State of each LED on the panel.
# For panels with less than 36 LEDs, the last array items are ignored.
rapha_interfaces/LedState[36] state
