# Reply to one BehaviorCommandMessage, matched by request_id.

int64 request_id

byte ACCEPTED = 0
byte REJECTED = 1
byte SUCCEEDED = 2
byte FAILED = 3
byte CANCELLED = 4
byte phase

string summary

bool rejected false
