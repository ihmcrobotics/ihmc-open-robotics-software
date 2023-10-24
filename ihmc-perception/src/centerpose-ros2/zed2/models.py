from enum import Enum

Models_path = "/root/centerpose-ros2/models"

class CenterPoseModels(Enum):
    BIKE = Models_path + "/bike_v1_140.pth"
    BOOK = Models_path + "/book_v1_140.pth"
    BOTTLE = Models_path + "/bottle_v1_sym_12_140.pth"
    CAMERA = Models_path + "/camera_v1_140.pth"
    CEREAL = Models_path + "/cereal_box_v1_140.pth"
    CHAIR = Models_path + "/chair_v1_140.pth"
    CUP = Models_path + "/cup_cup_v1_sym_12_140"
    MUG = Models_path + "/cup_mug_v1_140.pth"
    LAPTOP = Models_path + "/laptop_v1_140.pth"
    SHOE = Models_path + "/shoe_v1_140.pth"

class CenterPoseTrackModels(Enum):
    MUG = Models_path + "/cup_mug_15.pth"

class archType(Enum):
    NOTRACKING = 'dlav1_34'
    TRACKING = 'dla_34'

class experiment_type(Enum):
    LIVE = 1
    VIDEO = 2

if __name__ == '__main__':
    my_model = CenterPoseModels.MUG
    print(my_model)
    print(my_model.value)