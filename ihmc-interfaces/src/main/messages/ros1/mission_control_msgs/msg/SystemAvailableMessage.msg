
# The hostname of the system
# Equivalent of running `hostname` at the shell
string hostname

# A random ID for the topic name for messages from this machine
# We do not use the hostname because we don't want to assume the hostname is normalized
# to the format of topic names (i.e. hostnames might contain spaces, end with numbers, etc)
# Generated in ihmc-high-level-behaviors/src/mission-control/java/us/ihmc/missionControl/MissionControlDaemon.java
string instance_id


