# An instantaneous point on the time-line.
# Corresponds to java.time.Instant. Read about it:
# https://docs.oracle.com/en/java/javase/17/docs/api/java.base/java/time/Instant.html

# Seconds elapsed since the standard Java epoch of 1970-01-01T00:00:00Z
int64 seconds_since_epoch
# Additional nanoseconds which will always be between 0 and 999,999,999
int64 additional_nanos