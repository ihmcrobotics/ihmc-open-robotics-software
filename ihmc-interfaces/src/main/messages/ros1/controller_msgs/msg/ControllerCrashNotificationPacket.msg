int8 CONTROLLER_READ=0

int8 CONTROLLER_WRITE=1

int8 CONTROLLER_RUN=2

int8 ESTIMATOR_READ=3

int8 ESTIMATOR_WRITE=4

int8 ESTIMATOR_RUN=5

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 controller_crash_location

string stacktrace


