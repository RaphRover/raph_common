# Represents the state of the whole LED panel.

# Which LED panel does this state applies to.
raph_interfaces/LedPanel panel

# State of each LED on the panel.
# For panels with less than 40 LEDs, the last array items are ignored.
raph_interfaces/LedState[40] state
