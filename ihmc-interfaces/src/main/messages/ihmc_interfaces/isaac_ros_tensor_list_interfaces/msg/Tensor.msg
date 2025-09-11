# Name of the tensor
string name

# Shape information for tensor
TensorShape shape

# Data type for tensor
# Use the following values to represent these data types:
# -  1: "int8"
# -  2: "uint8"
# -  3: "int16"
# -  4: "uint16"
# -  5: "int32"
# -  6: "uint32"
# -  7: "int64"
# -  8: "uint64"
# -  9: "float32"
# - 10: "float64"
int32 data_type

# Strides of tensor (byte size for each dimension)
uint64[] strides

# Data buffer
uint8[] data
