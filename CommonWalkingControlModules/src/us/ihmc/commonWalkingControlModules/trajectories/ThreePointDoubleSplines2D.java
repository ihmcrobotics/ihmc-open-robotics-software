package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

public class ThreePointDoubleSplines2D
{
   private final Point3d pointOne = new Point3d();
   private final Point3d pointTwo = new Point3d();
   private final Point3d pointThree = new Point3d();
   
   public double getZ(Point2d queryPoint)
   {
      return pointOne.getZ();
   }

   public void setPoints(Point3d pointOne, Point3d pointTwo, Point3d pointThree)
   {
      this.pointOne.set(pointOne);
      this.pointTwo.set(pointTwo);
      this.pointThree.set(pointThree);   
   }

}
