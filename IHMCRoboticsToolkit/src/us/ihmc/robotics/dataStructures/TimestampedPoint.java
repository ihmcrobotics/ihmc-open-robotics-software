package us.ihmc.robotics.dataStructures;

import javax.vecmath.Point3f;

public class TimestampedPoint extends Point3f
{
   final long timestamp;
//   public ColorRGBA color;

   public TimestampedPoint(float x, float y, float z, long t)
   {
      super(x, y, z);
      timestamp = t;
   }
}
