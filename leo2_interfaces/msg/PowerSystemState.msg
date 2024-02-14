# Represents the state of the Power System

# Whether the charger is connected
bool charger_connected

# Whether the batteries are connected
bool bat1_connected
bool bat2_connected

# The states of the batteries, not valid for unconnected batteries
leo2_interfaces/BatteryState bat1_state
leo2_interfaces/BatteryState bat2_state

# The current power output in watts
float64 power

# The energy consumed in watt-hours since the robot was powered on
float64 energy
