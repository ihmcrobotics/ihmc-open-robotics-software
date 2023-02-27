# Big video packet. Supports up to RGB 8-bit 4K uncompressed images.

int64 acquisition_time_seconds_since_epoch
int64 acquisition_time_additional_nanos
uint16 image_width
uint16 image_height
byte[<=25000000] data
