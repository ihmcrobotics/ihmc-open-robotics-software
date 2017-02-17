package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.shapes.Plane3d;

public class HeightMapBestFitPlaneCalculator
{
   private List<Point3D> pointList = new ArrayList<Point3D>();

   private final PlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   public HeightMapBestFitPlaneCalculator()
   {
   }

   public Plane3d calculatePlane(HeightMapWithPoints heightMap, Point2D center, double kernelSizeX, double kernelSizeY) throws InsufficientDataException
   {
      if ((kernelSizeX == 0) || (kernelSizeY == 0))
         throw new RuntimeException("empty search region.");
      pointList = heightMap.getAllPointsWithinArea(center.getX(), center.getY(), kernelSizeX, kernelSizeY);

      Plane3d ret = new Plane3d();
      planeFitter.fitPlaneToPoints(center, pointList, ret);
      return ret;
   }

   public Plane3d calculatePlane(HeightMapWithPoints heightMap, Point2D center, double xExtent, double yExtent, InclusionFunction<Point3D> kernelMask)
           throws InsufficientDataException
   {
      if ((xExtent == 0) || (yExtent == 0))
         throw new RuntimeException("empty search region.");
      pointList = heightMap.getAllPointsWithinArea(center.getX(), center.getY(), xExtent, yExtent, kernelMask);

      Plane3d ret = new Plane3d();
      planeFitter.fitPlaneToPoints(center, pointList, ret);
      return ret;
   }



   public List<Point3D> getPointList()
   {
      return this.pointList;
   }

}
