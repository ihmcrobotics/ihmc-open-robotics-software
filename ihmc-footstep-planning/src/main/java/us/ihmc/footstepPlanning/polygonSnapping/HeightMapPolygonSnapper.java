package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.heightMap.HeightMapData;
import us.ihmc.robotics.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.createTransformToMatchSurfaceNormalPreserveX;
import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.setTranslationSettingZAndPreservingXAndY;

public class HeightMapPolygonSnapper
{
   private final List<Point3D> pointsInsidePolyon = new ArrayList<>();
   private final Plane3D bestFitPlane = new Plane3D();
   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   private double rootMeanSquaredError;
   private double area;

   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap, HeightMapData heightMap)
   {
      return snapPolygonToHeightMap(polygonToSnap, heightMap, Double.MAX_VALUE, -Double.MAX_VALUE);
   }

   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap, HeightMapData heightMap, double snapHeightThreshold)
   {
      return snapPolygonToHeightMap(polygonToSnap, heightMap, snapHeightThreshold, -Double.MAX_VALUE);
   }

   /**
    * Snaps the given polygon to the height map by a least-squares plane fit.
    *
    * - Any cells with heights below minimumHeightToConsider are ignored.
    * - Any cells with heights below maxZ - snapHeightThreshold are ignored, where maxZ is the max height within the polygon
    */
   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap, HeightMapData heightMap, double snapHeightThreshold, double minimumHeightToConsider)
   {
      double areaPerCell = MathTools.square(heightMap.getGridResolutionXY());
      double epsilonDistance = Math.sqrt(0.5) * heightMap.getGridResolutionXY();

      pointsInsidePolyon.clear();
      bestFitPlane.setToNaN();
      Point2D gridCenter = heightMap.getGridCenter();

      int centerIndex = HeightMapTools.computeCenterIndex(heightMap.getGridSizeXY(), heightMap.getGridResolutionXY());
      int minIndexX = HeightMapTools.coordinateToIndex(polygonToSnap.getMinX(), gridCenter.getX(), heightMap.getGridResolutionXY(), centerIndex);
      int maxIndexX = HeightMapTools.coordinateToIndex(polygonToSnap.getMaxX(), gridCenter.getX(), heightMap.getGridResolutionXY(), centerIndex);
      int minIndexY = HeightMapTools.coordinateToIndex(polygonToSnap.getMinY(), gridCenter.getY(), heightMap.getGridResolutionXY(), centerIndex);
      int maxIndexY = HeightMapTools.coordinateToIndex(polygonToSnap.getMaxY(), gridCenter.getY(), heightMap.getGridResolutionXY(), centerIndex);

      for (int xIndex = minIndexX; xIndex <= maxIndexX; xIndex++)
      {
         for (int yIndex = minIndexY; yIndex <= maxIndexY; yIndex++)
         {
            double height = heightMap.getHeightAt(xIndex, yIndex);

            if (Double.isNaN(height) || height < minimumHeightToConsider)
            {
               continue;
            }

            double x = HeightMapTools.indexToCoordinate(xIndex, gridCenter.getX(), heightMap.getGridResolutionXY(), centerIndex);
            double y = HeightMapTools.indexToCoordinate(yIndex, gridCenter.getY(), heightMap.getGridResolutionXY(), centerIndex);

            if (polygonToSnap.distance(new Point2D(x, y)) > epsilonDistance)
            {
               continue;
            }

            pointsInsidePolyon.add(new Point3D(x, y, height));
         }
      }

      if (pointsInsidePolyon.isEmpty())
      {
         return null;
      }

      double maxZ = pointsInsidePolyon.stream().mapToDouble(Point3D::getZ).max().getAsDouble();
      double minZ = maxZ - snapHeightThreshold;
      pointsInsidePolyon.removeIf(point -> point.getZ() < minZ);
      area = pointsInsidePolyon.size() * areaPerCell;

      if (pointsInsidePolyon.size() < 3)
      {
         return null;
      }

      planeFitter.fitPlaneToPoints(pointsInsidePolyon, bestFitPlane);
      rootMeanSquaredError = 0.0;

      for (int i = 0; i < pointsInsidePolyon.size(); i++)
      {
         double predictedHeight = bestFitPlane.getZOnPlane(pointsInsidePolyon.get(i).getX(), pointsInsidePolyon.get(i).getY());
         rootMeanSquaredError += MathTools.square(predictedHeight - pointsInsidePolyon.get(i).getZ());
      }

      if (bestFitPlane.containsNaN())
      {
         return null;
      }

      rootMeanSquaredError = Math.sqrt(rootMeanSquaredError / pointsInsidePolyon.size());

      RigidBodyTransform transformToReturn = createTransformToMatchSurfaceNormalPreserveX(bestFitPlane.getNormal());

      Point2DReadOnly centroid = polygonToSnap.getCentroid();
      double height = bestFitPlane.getZOnPlane(centroid.getX(), centroid.getY());
      setTranslationSettingZAndPreservingXAndY(new Point3D(centroid.getX(), centroid.getY(), height), transformToReturn);

      return transformToReturn;
   }

   public Plane3D getBestFitPlane()
   {
      return bestFitPlane;
   }

   public double getRMSError()
   {
      return rootMeanSquaredError;
   }

   public double getArea()
   {
      return area;
   }
}
