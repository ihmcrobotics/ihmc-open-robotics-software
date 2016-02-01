package us.ihmc.robotics.dataStructures;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public class Pose
{
   private final Point3d position;
   private final Quat4d orientation;
   public Pose(Point3d position, Quat4d orientation)
   {
      this.position = position;
      this.orientation = orientation;
   }
   
   public Point3d getPoint()
   {
      return position;
   }
   
   public Quat4d getOrientation()
   {
      return orientation;
   }

}
