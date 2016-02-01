# DEPRECATED. Use pr2_msgs/BatteryServer2 instead.
Header header
uint32 MAX_BAT_COUNT=4
uint32 MAX_BAT_REG=48
int32 id  # unique ID for each battery server
# Battery System Stats
int32 lastTimeSystem #epoch time
uint16 timeLeft      # in minutes
uint16 averageCharge # in percent
string message
# Battery Controller Flags
int32 lastTimeController #epoch time
uint16 present
uint16 charging
uint16 discharging
uint16 reserved
uint16 powerPresent
uint16 powerNG
uint16 inhibited
# for each battery
pr2_msgs/BatteryState[] battery
