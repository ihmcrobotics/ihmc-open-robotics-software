# The name of the systemd service
string service_name

# The status from systemctl status <service_name>
string status

# If the log_data is complete (a full log)
bool refresh

# log_data contains packed system log strings. It may
# be empty most of the time if there are no new log lines
# generated by the service
byte[<=25000000] log_data
