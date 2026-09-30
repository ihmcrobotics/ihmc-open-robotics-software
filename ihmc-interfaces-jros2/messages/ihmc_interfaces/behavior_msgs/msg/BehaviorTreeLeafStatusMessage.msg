# Leaf flags for a behavior tree that is already running.
# A snapshot replaces every leaf. Otherwise only the listed leaves changed.
# leaf_index is the depth-first leaf index, the same on any side that loaded tree_file.

bool snapshot false

# File name passed to start(Tree). Empty when this is a patch, or when the tree was not loaded from a file.
string tree_file

uint32[<=120] leaf_index

bool[<=120] is_active

bool[<=120] is_next_for_execution

bool[<=120] can_execute

bool[<=120] is_executing

bool[<=120] failed

# Seconds the leaf has been executing. NaN when this leaf is not an action.
float64[<=120] elapsed_seconds
