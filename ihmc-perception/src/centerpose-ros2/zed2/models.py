from enum import Enum
from dataclasses import dataclass
import argparse

Models_path = "/root/centerpose-ros2"

@dataclass
class CenterposeObjectClass():
    id : int
    modelPath : str
    trackingModelPath : str
    modelScale : float

    def getID(self):
        return self.id

    def getModelPath(self):
        return Models_path + "/models" + self.modelPath

    def getTrackingModelPath(self):
        return Models_path + "/models" + self.trackingModelPath

    def getModelScale(self):
        return self.modelScale

class ArgTypeMixin(Enum):

    @classmethod
    def argtype(cls, s: str) -> Enum:
        try:
            return cls[s]
        except KeyError:
            raise argparse.ArgumentTypeError(
                f"{s!r} is not a valid {cls.__name__}")

    def __str__(self):
        return self.name

class CenterPoseModels(ArgTypeMixin, Enum):
    BIKE = CenterposeObjectClass(0,"bike_v1_140.pth", "", 1.0)
    BOOK = CenterposeObjectClass(1, "/book_v1_140.pth", "", 1.0)
    BOTTLE = CenterposeObjectClass(2, "/bottle_v1_sym_12_140.pth", "", 1.0)
    CAMERA = CenterposeObjectClass(3, "/camera_v1_140.pth", "", 1.0)
    CEREAL = CenterposeObjectClass(4, "/cereal_box_v1_140.pth", "", 1.0)
    CHAIR = CenterposeObjectClass(5, "/chair_v1_140.pth", "", 1.27)
    CUP = CenterposeObjectClass(6, "/cup_cup_v1_sym_12_140", "", 1.0)
    MUG = CenterposeObjectClass(7, "/cup_mug_v1_140.pth", "/cup_mug_15.pth", 10.0)
    LAPTOP = CenterposeObjectClass(8, "/laptop_v1_140.pth", "", 1.0)
    SHOE = CenterposeObjectClass(9, "/shoe_v1_140.pth", "", 1.0)

class archType(Enum):
    NOTRACKING = 'dlav1_34'
    TRACKING = 'dla_34'

class experiment_type(Enum):
    LIVE = 1
    VIDEO = 2

if __name__ == '__main__':
    my_model = CenterPoseModels.MUG
    print(my_model.name)
    print(my_model.value)
    print(my_model.value.getID())
    print(my_model.value.getModelPath())
    print(my_model.value.getModelScale())