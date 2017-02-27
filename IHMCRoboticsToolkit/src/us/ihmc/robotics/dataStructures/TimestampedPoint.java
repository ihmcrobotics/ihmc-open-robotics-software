package us.ihmc.robotics.dataStructures;

import us.ihmc.euclid.tuple3D.Point3D32;

public class TimestampedPoint extends Point3D32
{
   private static final long serialVersionUID = 2910113387832828116L;
   
   final long timestamp;

   public TimestampedPoint(float x, float y, float z, long t)
   {
      super(x, y, z);
      timestamp = t;
   }
}
