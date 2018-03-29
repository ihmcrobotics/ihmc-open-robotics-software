# Message specific to Atlas, it is reported by the IHMC state estimator.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

float32[<=6] electric_joint_temperatures
float32[<=6] electric_joint_currents
bool[<=6] electric_joint_enabled_array

float32[15] raw_imu_timestamps
float32[15] raw_imu_packet_counts
geometry_msgs/Vector3[15] raw_imu_rates
geometry_msgs/Vector3[15] raw_imu_deltas

bool battery_charging
float32 battery_voltage
float32 battery_current
float32 remaining_battery_time
float32 remaining_amp_hours
float32 remaining_charge_percentage
int64 battery_cycle_count

float32 pump_inlet_pressure
float32 pump_supply_pressure
float32 air_sump_pressure
float32 pump_supply_temperature
float32 pump_rpm
float32 motor_temperature
float32 motor_driver_temperature
