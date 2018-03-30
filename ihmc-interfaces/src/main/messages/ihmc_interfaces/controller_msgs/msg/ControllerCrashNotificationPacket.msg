# Message notifying if the IHMC whole-body controller has crashed unexpectedly.

byte CONTROLLER_READ = 0
byte CONTROLLER_WRITE = 1
byte CONTROLLER_RUN = 2
byte ESTIMATOR_READ = 3
byte ESTIMATOR_WRITE = 4
byte ESTIMATOR_RUN = 5

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte controller_crash_location 255
string stacktrace