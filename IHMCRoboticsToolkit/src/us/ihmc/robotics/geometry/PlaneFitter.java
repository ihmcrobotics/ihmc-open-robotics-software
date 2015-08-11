package us.ihmc.robotics.geometry;

import us.ihmc.robotics.geometry.shapes.Plane3d;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import java.util.List;

public interface PlaneFitter
{
   /**
    *
    * @param pointList The list of points to fit
    * @param planeToPack the plane to pack
    * @return the average error of the points to the plane fit
    */
   public abstract double fitPlaneToPoints(List<Point3d> pointList, Plane3d planeToPack);

   /**
    *
    * @param center origin the plane at the center if possible
    * @param pointList The list of points to fit
    * @param planeToPack the plane to pack
    * @return the average error of the points to the plane fit
    */
   public abstract double fitPlaneToPoints(Point2d center, List<Point3d> pointList, Plane3d planeToPack);
}
