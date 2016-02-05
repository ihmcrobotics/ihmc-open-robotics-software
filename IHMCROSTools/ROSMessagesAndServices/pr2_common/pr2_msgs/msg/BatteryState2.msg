# This message communicates the state of a single battery.
# Battery Controller Flags, one per battery
bool present       # is this pack present
bool charging      # is this pack charging
bool discharging   # is this pack discharging
bool power_present # is there an input voltage
bool power_no_good # is there a fault (No Good)
bool inhibited     # is this pack disabled for some reason
# These registers are per battery
time      last_battery_update     # last time any battery update occurred
int16[48] battery_register        # value of this register in the battery
bool[48]  battery_update_flag     # Has this register ever been updated
time[48]  battery_register_update # last time this specific register was updated
