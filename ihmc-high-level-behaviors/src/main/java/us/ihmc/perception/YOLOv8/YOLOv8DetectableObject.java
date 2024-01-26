package us.ihmc.perception.YOLOv8;

import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

public enum YOLOv8DetectableObject
{
   PERSON,
   BICYCLE,
   CAR,
   MOTORCYCLE,
   AIRPLANE,
   BUS,
   TRAIN,
   TRUCK,
   BOAT,
   TRAFFIC_LIGHT,
   FIRE_HYDRANT,
   STOP_SIGN,
   PARKING_METER,
   BENCH,
   BIRD,
   CAT,
   DOG,
   HORSE,
   SHEEP,
   COW,
   ELEPHANT,
   BEAR,
   ZEBRA,
   GIRAFFE,
   BACKPACK,
   UMBRELLA,
   HANDBAG,
   TIE,
   SUITCASE,
   FRISBEE,
   SKIS,
   SNOWBOARD,
   SPORTS_BALL,
   KITE,
   BASEBALL_BAT,
   BASEBALL_GLOVE,
   SKATEBOARD,
   SURFBOARD,
   TENNIS_RACKET,
   BOTTLE,
   WINE_GLASS,
   CUP,
   FORK,
   KNIFE,
   SPOON,
   BOWL,
   BANANA,
   APPLE,
   SANDWICH,
   ORANGE,
   BROCCOLI,
   CARROT,
   HOT_DOG,
   PIZZA,
   DONUT,
   CAKE,
   CHAIR,
   COUCH,
   POTTED_PLANT,
   BED,
   DINING_TABLE,
   TOILET,
   TV,
   LAPTOP,
   MOUSE,
   REMOTE,
   KEYBOARD,
   CELL_PHONE,
   MICROWAVE,
   OVEN,
   TOASTER,
   SINK,
   REFRIGERATOR,
   BOOK,
   CLOCK,
   VASE,
   SCISSORS,
   TEDDY_BEAR,
   HAIR_DRIER,
   TOOTHBRUSH
   ;

   public int getClassId()
   {
      return ordinal();
   }

   PrimitiveRigidBodyShape correspondingShape;

   @Override
   public String toString()
   {
      return name().toLowerCase().replaceAll("_", " ");
   }
}
