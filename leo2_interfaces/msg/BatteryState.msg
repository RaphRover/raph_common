# Represents the state of a single battery

# The current voltage in volts
float64 voltage

# Estimated state of charge in percents
float64 state_of_charge

# The target battery mode
leo2_interfaces/BatteryMode target_mode

# The actual battery mode
leo2_interfaces/BatteryMode current_mode
