# One request to start or cancel a mimic, a person follow, or a saved behavior tree.
# The runtime applies this on the behavior thread and answers on BehaviorResultMessage.

int64 request_id

byte MIMIC = 0
byte FOLLOW = 1
byte TREE = 2
byte CANCEL = 3
byte kind

# Mimic catalog name, follow target label, or behavior-tree file. Ignored for CANCEL.
string name

# Follow only: lock the nearest person instead of a described target.
bool closest_person false

# A tree leaf calling mimic or follow. Does not cancel the behavior that is already running.
bool nested false
