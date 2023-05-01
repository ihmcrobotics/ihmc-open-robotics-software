# This message is part of the Exoskeleton Metabolic Estimation Interface
# This message acts as a go-between between Java and Python.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Current metabolic rate plus the change from the previous metabolic rate
float64 metabolic_rate

float64 delta_metabolic_rate


