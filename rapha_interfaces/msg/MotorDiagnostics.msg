std_msgs/Header header
uint8[6] temperature
float64[6] power
float64[6] energy

uint8 FAULT_BIT_SENSOR_FAULT=1
uint8 FAULT_BIT_BUS_OVERCURRENT=2
uint8 FAULT_BIT_PHASE_OVERCURRENT=4
uint8 FAULT_BIT_STALL=8
uint8 FAULT_BIT_OVERHEAT=16

uint8[6] fault_mask_active
uint8[6] fault_mask_occured