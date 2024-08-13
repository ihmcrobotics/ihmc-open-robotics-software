#Message used for syncing the body dimension config files in eva's resources with eva's local config files

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int8[<=2048] file_name


# This is the wrench at the foot, with respect to the sole frame
float64 thigh_length
float64 shank_length
uint32 shoe_size_oridinal