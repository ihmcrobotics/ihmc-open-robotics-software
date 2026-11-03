# One request to start a mimic, a person follow, or a saved behavior tree, or to cancel the active one.

int64 request_id

byte MIMIC = 0
byte FOLLOW = 1
byte TREE = 2
byte CANCEL = 3
byte kind

# Mimic catalog name, follow target label, or behavior-tree file such as skill/GoHome.json.
string name

# Follow only. Lock the nearest person instead of the label in name.
bool closest_person false
