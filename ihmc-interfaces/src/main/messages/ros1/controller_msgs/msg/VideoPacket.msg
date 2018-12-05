int8 VIDEO_SOURCE_MULTISENSE_LEFT_EYE=0

int8 VIDEO_SOURCE_MULTISENSE_RIGHT_EYE=1

int8 VIDEO_SOURCE_FISHEYE_LEFT=2

int8 VIDEO_SOURCE_FISHEYE_RIGHT=3

int8 VIDEO_SOURCE_CV_THRESHOLD=4

int8 VIDEO_SOURCE_IMAGE_PROCESSING_BEHAVIOR=5

int8 VIDEO_SOURCE_AWARE_FACE_TRACKER=6

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 video_source

int64 timestamp

int8[] data

geometry_msgs/Point position

geometry_msgs/Quaternion orientation

controller_msgs/IntrinsicParametersMessage intrinsic_parameters


