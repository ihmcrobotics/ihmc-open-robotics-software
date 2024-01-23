package us.ihmc.perception.YOLOv8;

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
   ;

   public int getClassId()
   {
      return ordinal();
   }

   @Override
   public String toString()
   {
      return name().toLowerCase().replaceAll("_", " ");
   }
}
