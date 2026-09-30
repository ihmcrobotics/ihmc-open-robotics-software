# Answer for one BehaviorCommandMessage, matched by request_id.
# Speech waits for ACCEPTED, REJECTED, or CANCELLED. Later phases are for a tree leaf.

int64 request_id

byte ACCEPTED = 0
byte REJECTED = 1
byte RUNNING = 2
byte SUCCEEDED = 3
byte FAILED = 4
byte CANCELLED = 5
byte phase

string summary

# True only for REJECTED. Speech maps this onto CommandExecutionResult.stubbed.
bool rejected false
