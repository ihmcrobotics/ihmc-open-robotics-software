package us.ihmc.perception.YOLOv8;

import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import javax.annotation.Nullable;

public enum YOLOv8DetectableObject
{
   PERSON(null),
   BICYCLE(PrimitiveRigidBodyShape.CUSTOM),
   CAR(null),
   MOTORCYCLE(null),
   AIRPLANE(PrimitiveRigidBodyShape.ELLIPSOID),
   BUS(PrimitiveRigidBodyShape.BOX),
   TRAIN(PrimitiveRigidBodyShape.BOX),
   TRUCK(null),
   BOAT(PrimitiveRigidBodyShape.ELLIPSOID),
   TRAFFIC_LIGHT(PrimitiveRigidBodyShape.BOX),
   FIRE_HYDRANT(PrimitiveRigidBodyShape.CYLINDER),
   STOP_SIGN(null),
   PARKING_METER(PrimitiveRigidBodyShape.BOX),
   BENCH(null),
   BIRD(PrimitiveRigidBodyShape.ELLIPSOID),
   CAT(null),
   DOG(null),
   HORSE(null),
   SHEEP(null),
   COW(null),
   ELEPHANT(null),
   BEAR(null),
   ZEBRA(null),
   GIRAFFE(null),
   BACKPACK(PrimitiveRigidBodyShape.ELLIPSOID),
   UMBRELLA(null),
   HANDBAG(PrimitiveRigidBodyShape.BOX),
   TIE(null),
   SUITCASE(PrimitiveRigidBodyShape.BOX),
   FRISBEE(PrimitiveRigidBodyShape.ELLIPSOID),
   SKIS(null),
   SNOWBOARD(null),
   SPORTS_BALL(PrimitiveRigidBodyShape.ELLIPSOID),
   KITE(null),
   BASEBALL_BAT(PrimitiveRigidBodyShape.CYLINDER),
   BASEBALL_GLOVE(null),
   SKATEBOARD(null),
   SURFBOARD(null),
   TENNIS_RACKET(null),
   BOTTLE(PrimitiveRigidBodyShape.CYLINDER),
   WINE_GLASS(null),
   CUP(PrimitiveRigidBodyShape.CYLINDER),
   FORK(null),
   KNIFE(null),
   SPOON(null),
   BOWL(PrimitiveRigidBodyShape.ELLIPSOID),
   BANANA(PrimitiveRigidBodyShape.ELLIPSOID),
   APPLE(PrimitiveRigidBodyShape.ELLIPSOID),
   SANDWICH(PrimitiveRigidBodyShape.BOX),
   ORANGE(PrimitiveRigidBodyShape.ELLIPSOID),
   BROCCOLI(null),
   CARROT(PrimitiveRigidBodyShape.CONE),
   HOT_DOG(PrimitiveRigidBodyShape.CYLINDER),
   PIZZA(PrimitiveRigidBodyShape.ELLIPSOID),
   DONUT(PrimitiveRigidBodyShape.ELLIPSOID),
   CAKE(PrimitiveRigidBodyShape.CYLINDER),
   CHAIR(null),
   COUCH(null),
   POTTED_PLANT(null),
   BED(PrimitiveRigidBodyShape.BOX),
   DINING_TABLE(null),
   TOILET(null),
   TV(PrimitiveRigidBodyShape.BOX),
   LAPTOP(PrimitiveRigidBodyShape.BOX),
   MOUSE(PrimitiveRigidBodyShape.ELLIPSOID),
   REMOTE(PrimitiveRigidBodyShape.BOX),
   KEYBOARD(PrimitiveRigidBodyShape.BOX),
   CELL_PHONE(PrimitiveRigidBodyShape.BOX),
   MICROWAVE(PrimitiveRigidBodyShape.BOX),
   OVEN(PrimitiveRigidBodyShape.BOX),
   TOASTER(PrimitiveRigidBodyShape.BOX),
   SINK(null),
   REFRIGERATOR(PrimitiveRigidBodyShape.BOX),
   BOOK(PrimitiveRigidBodyShape.BOX),
   CLOCK(PrimitiveRigidBodyShape.CYLINDER),
   VASE(PrimitiveRigidBodyShape.ELLIPSOID),
   SCISSORS(null),
   TEDDY_BEAR(null),
   HAIR_DRIER(null),
   TOOTHBRUSH(null)
   ;

   @Nullable
   private PrimitiveRigidBodyShape correspondingShape;

   YOLOv8DetectableObject(PrimitiveRigidBodyShape correspondingShape)
   {
      this.correspondingShape = correspondingShape;
   }

   /**
    * The corresponding shape is the best primitive approximation of the object determined by Tomasz.
    * For example, a bird is just an ellipsoid, a train is a long box, and a carrot is just a cone.
    * If you disagree, feel free to change the corresponding shape (although you're clearly wrong).
    * @return the objectively correct approximation of the object as a primitive shape. Null if no approximation is good.
    */
   public PrimitiveRigidBodyShape getCorrespondingShape()
   {
      return correspondingShape;
   }

   public int getClassId()
   {
      return ordinal();
   }

   public static YOLOv8DetectableObject fromByte(byte enumAsByte)
   {
      return values()[enumAsByte];
   }

   public byte toByte()
   {
      return (byte) this.ordinal();
   }
   @Override
   public String toString()
   {
      return name().toLowerCase().replaceAll("_", " ");
   }
}