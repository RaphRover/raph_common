# Represents the state of the whole LED panel.

uint8 PANEL_FRONT=0
uint8 PANEL_REAR=1
uint8 PANEL_LEFT=2
uint8 PANEL_RIGHT=3

# Which LED panel does this state applies to.
uint8 panel

# State of each LED on the panel.
# The largest panel has 40 LEDs. If the length of the array is smaller than the number of LEDs on
# the panel, the remaining LEDs are not changed. If the length is greater than the number of LEDs
# on the panel, the extra states are ignored.
raph_interfaces/LedState[<=40] state
