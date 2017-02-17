package us.ihmc.robotics.dataStructures;

import us.ihmc.euclid.tuple3D.Point3D32;

public class TimestampedPoint extends Point3D32
{
   final long timestamp;
//   public ColorRGBA color;

   public TimestampedPoint(float x, float y, float z, long t)
   {
      super(x, y, z);
      timestamp = t;
   }
}
