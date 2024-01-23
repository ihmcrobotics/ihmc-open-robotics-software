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
   ELEPHANT
   // TODO: Finish adding the detectable object types
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
