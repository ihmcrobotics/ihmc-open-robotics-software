package us.ihmc.robotics.geometry;

import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.shapes.Plane3d;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;

public class HeightMapBestFitPlaneCalculator
{
   private List<Point3d> pointList = new ArrayList<Point3d>();

   private final PlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   public HeightMapBestFitPlaneCalculator()
   {
   }

   public Plane3d calculatePlane(HeightMapWithPoints heightMap, Point2d center, double kernelSizeX, double kernelSizeY) throws InsufficientDataException
   {
      if ((kernelSizeX == 0) || (kernelSizeY == 0))
         throw new RuntimeException("empty search region.");
      pointList = heightMap.getAllPointsWithinArea(center.getX(), center.getY(), kernelSizeX, kernelSizeY);

      Plane3d ret = new Plane3d();
      planeFitter.fitPlaneToPoints(center, pointList, ret);
      return ret;
   }

   public Plane3d calculatePlane(HeightMapWithPoints heightMap, Point2d center, double xExtent, double yExtent, InclusionFunction<Point3d> kernelMask)
           throws InsufficientDataException
   {
      if ((xExtent == 0) || (yExtent == 0))
         throw new RuntimeException("empty search region.");
      pointList = heightMap.getAllPointsWithinArea(center.getX(), center.getY(), xExtent, yExtent, kernelMask);

      Plane3d ret = new Plane3d();
      planeFitter.fitPlaneToPoints(center, pointList, ret);
      return ret;
   }



   public List<Point3d> getPointList()
   {
      return this.pointList;
   }

}
