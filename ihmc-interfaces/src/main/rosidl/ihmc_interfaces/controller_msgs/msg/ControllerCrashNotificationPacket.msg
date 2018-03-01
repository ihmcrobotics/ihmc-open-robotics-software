# Message notifying if the IHMC whole-body controller has crashed unexpectedly.

byte CONTROLLER_READ = 0
byte CONTROLLER_WRITE = 1
byte CONTROLLER_RUN = 2
byte ESTIMATOR_READ = 3
byte ESTIMATOR_WRITE = 4
byte ESTIMATOR_RUN = 5

byte controller_crash_location -1
string stacktrace