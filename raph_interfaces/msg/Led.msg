# Represents the position and the state of the LED.

uint8 PANEL_FRONT=0
uint8 PANEL_REAR=1
uint8 PANEL_LEFT=2
uint8 PANEL_RIGHT=3

# The panel the LED is on.
uint8 panel

# The id of the LED on the panel.
uint8 id

# The state of the LED.
raph_interfaces/LedState state