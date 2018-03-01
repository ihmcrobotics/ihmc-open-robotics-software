# Used to determine which of the next *_value fields are set.
# ParameterType.PARAMETER_NOT_SET indicates that the parameter was not set
# (if gotten) or is uninitialized.
# Values are enumerated in ParameterType.msg

uint8 type

# "Variant" style storage of the parameter value.
bool bool_value
int64 integer_value
float64 double_value
string string_value
byte[] bytes_value
