package us.ihmc.perception.YOLOv8;

import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import javax.annotation.Nullable;

public enum YOLOv8DetectableObject
{
   PERSON(null, null),
   BICYCLE(PrimitiveRigidBodyShape.CUSTOM, null),
   CAR(null, null),
   MOTORCYCLE(null, null),
   AIRPLANE(PrimitiveRigidBodyShape.ELLIPSOID, null),
   BUS(PrimitiveRigidBodyShape.BOX, null),
   TRAIN(PrimitiveRigidBodyShape.BOX, null),
   TRUCK(null, null),
   BOAT(PrimitiveRigidBodyShape.ELLIPSOID, null),
   TRAFFIC_LIGHT(PrimitiveRigidBodyShape.BOX, null),
   FIRE_HYDRANT(PrimitiveRigidBodyShape.CYLINDER, null),
   STOP_SIGN(null, null),
   PARKING_METER(PrimitiveRigidBodyShape.BOX, null),
   BENCH(null, null),
   BIRD(PrimitiveRigidBodyShape.ELLIPSOID, null),
   CAT(null, null),
   DOG(null, null),
   HORSE(null, null),
   SHEEP(null, null),
   COW(null, null),
   ELEPHANT(null, null),
   BEAR(null, null),
   ZEBRA(null, null),
   GIRAFFE(null, null),
   BACKPACK(PrimitiveRigidBodyShape.ELLIPSOID, null),
   UMBRELLA(null,null),
   HANDBAG(PrimitiveRigidBodyShape.BOX, null),
   TIE(null, null),
   SUITCASE(PrimitiveRigidBodyShape.BOX, null),
   FRISBEE(PrimitiveRigidBodyShape.ELLIPSOID, null),
   SKIS(null, null),
   SNOWBOARD(null, null),
   SPORTS_BALL(PrimitiveRigidBodyShape.ELLIPSOID, null),
   KITE(null, null),
   BASEBALL_BAT(PrimitiveRigidBodyShape.CYLINDER, null),
   BASEBALL_GLOVE(null, null),
   SKATEBOARD(null, null),
   SURFBOARD(null, null),
   TENNIS_RACKET(null, null),
   BOTTLE(PrimitiveRigidBodyShape.CYLINDER, null),
   WINE_GLASS(null, null),
   CUP(PrimitiveRigidBodyShape.CYLINDER, "ihmc_mug_points.csv"),
   FORK(null, null),
   KNIFE(null, null),
   SPOON(null, null),
   BOWL(PrimitiveRigidBodyShape.ELLIPSOID, null),
   BANANA(PrimitiveRigidBodyShape.ELLIPSOID, null),
   APPLE(PrimitiveRigidBodyShape.ELLIPSOID, null),
   SANDWICH(PrimitiveRigidBodyShape.BOX, null),
   ORANGE(PrimitiveRigidBodyShape.ELLIPSOID, null),
   BROCCOLI(null, null),
   CARROT(PrimitiveRigidBodyShape.CONE, null),
   HOT_DOG(PrimitiveRigidBodyShape.CYLINDER, null),
   PIZZA(PrimitiveRigidBodyShape.ELLIPSOID, null),
   DONUT(PrimitiveRigidBodyShape.ELLIPSOID, null),
   CAKE(PrimitiveRigidBodyShape.CYLINDER, null),
   CHAIR(null, null),
   COUCH(null, null),
   POTTED_PLANT(null, null),
   BED(PrimitiveRigidBodyShape.BOX, null),
   DINING_TABLE(null, null),
   TOILET(null, null),
   TV(PrimitiveRigidBodyShape.BOX, null),
   LAPTOP(PrimitiveRigidBodyShape.BOX, null),
   MOUSE(PrimitiveRigidBodyShape.ELLIPSOID, null),
   REMOTE(PrimitiveRigidBodyShape.BOX, null),
   KEYBOARD(PrimitiveRigidBodyShape.BOX, null),
   CELL_PHONE(PrimitiveRigidBodyShape.BOX, null),
   MICROWAVE(PrimitiveRigidBodyShape.BOX, null),
   OVEN(PrimitiveRigidBodyShape.BOX, null),
   TOASTER(PrimitiveRigidBodyShape.BOX, null),
   SINK(null, null),
   REFRIGERATOR(PrimitiveRigidBodyShape.BOX, null),
   BOOK(PrimitiveRigidBodyShape.BOX, null),
   CLOCK(PrimitiveRigidBodyShape.CYLINDER, null),
   VASE(PrimitiveRigidBodyShape.ELLIPSOID, null),
   SCISSORS(null, null),
   TEDDY_BEAR(null, null),
   HAIR_DRIER(null, null),
   TOOTHBRUSH(null, null)
   ;

   @Nullable
   private PrimitiveRigidBodyShape primitiveApproximation;

   private String pointCloudFileName;

   YOLOv8DetectableObject(PrimitiveRigidBodyShape primitiveApproximation, String pointCloudFileName)
   {
      this.primitiveApproximation = primitiveApproximation;
      this.pointCloudFileName = pointCloudFileName;
   }

   /**
    * The corresponding shape is the best primitive approximation of the object determined by Tomasz.
    * For example, a bird is just an ellipsoid, a train is a long box, and a carrot is just a cone.
    * If you disagree, feel free to change the corresponding shape (although you're clearly wrong).
    * @return the objectively correct approximation of the object as a primitive shape. Null if no approximation is good.
    */
   public PrimitiveRigidBodyShape getPrimitiveApproximation()
   {
      return primitiveApproximation;
   }

   public String getPointCloudFileName()
   {
      return pointCloudFileName;
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