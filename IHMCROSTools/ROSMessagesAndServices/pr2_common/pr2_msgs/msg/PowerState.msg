# This message communicates the state of the PR2's power system.
Header header
float64 power_consumption ## Watts
duration time_remaining ## estimated time to empty or full
string prediction_method ## how time_remaining is being calculated
int8  relative_capacity ## percent of capacity
int8  AC_present ## number of packs detecting AC power, > 0 means plugged in
