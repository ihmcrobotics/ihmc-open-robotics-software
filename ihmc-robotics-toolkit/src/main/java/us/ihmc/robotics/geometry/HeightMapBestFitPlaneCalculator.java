package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;

public class HeightMapBestFitPlaneCalculator
{
   private List<Point3D> pointList = new ArrayList<Point3D>();

   private final PlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   public HeightMapBestFitPlaneCalculator()
   {
   }

   public Plane3D calculatePlane(HeightMapWithPoints heightMap, Point2DBasics center, double kernelSizeX, double kernelSizeY) throws InsufficientDataException
   {
      if ((kernelSizeX == 0) || (kernelSizeY == 0))
         throw new RuntimeException("empty search region.");
      pointList = heightMap.getAllPointsWithinArea(center.getX(), center.getY(), kernelSizeX, kernelSizeY);

      Plane3D ret = new Plane3D();
      planeFitter.fitPlaneToPoints(center, pointList, ret);
      return ret;
   }

   public Plane3D calculatePlane(HeightMapWithPoints heightMap, Point2DBasics center, double xExtent, double yExtent, InclusionFunction<Point3D> kernelMask)
           throws InsufficientDataException
   {
      if ((xExtent == 0) || (yExtent == 0))
         throw new RuntimeException("empty search region.");
      pointList = heightMap.getAllPointsWithinArea(center.getX(), center.getY(), xExtent, yExtent, kernelMask);

      Plane3D ret = new Plane3D();
      planeFitter.fitPlaneToPoints(center, pointList, ret);
      return ret;
   }



   public List<Point3D> getPointList()
   {
      return this.pointList;
   }

}
