package us.ihmc.footstepPlanning.polygonSnapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.createTransformToMatchSurfaceNormalPreserveX;
import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.setTranslationSettingZAndPreservingXAndY;

public class HeightMapPolygonSnapper
{
   private final List<Point2D> footPointsInEnvironment = new ArrayList<>();
   private final List<Point3D> heightMapPointsInPolygon = new ArrayList<>();
   private final Plane3D bestFitPlane = new Plane3D();
   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   private double maxPossibleRMSError;
   private double rootMeanSquaredError;
   private double area;

   private double snapAreaResolution = 0.2;

   public void setSnapAreaResolution(double snapAreaResolution)
   {
      this.snapAreaResolution = snapAreaResolution;
   }

   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap, HeightMapData heightMap)
   {
      return snapPolygonToHeightMap(polygonToSnap, heightMap, Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE);
   }

   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap,
                                                    HeightMapData heightMap,
                                                    double snapHeightThreshold,
                                                    double minimumSurfaceInclineRadians)
   {
      return snapPolygonToHeightMap(polygonToSnap, heightMap, snapHeightThreshold, minimumSurfaceInclineRadians, -Double.MAX_VALUE);
   }

   /**
    * Snaps the given polygon to the height map by a least-squares plane fit.
    *
    * - Any cells with heights below minimumHeightToConsider are ignored.
    * - Any cells with heights below maxZ - snapHeightThreshold are ignored, where maxZ is the max height within the polygon
    */
   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap,
                                                    HeightMapData heightMap,
                                                    double snapHeightThreshold,
                                                    double minimumSurfaceInclineRadians,
                                                    double minimumHeightToConsider)
   {
      footPointsInEnvironment.clear();
      heightMapPointsInPolygon.clear();
      bestFitPlane.setToNaN();

      if (polygonToSnap.getNumberOfVertices() != 4)
         throw new RuntimeException("We aren't set up to use this");

      // Here, we want to go through the grid under the foot and record the points that may be in contact with the foot. This is better for plane fitting
      // the actual values. If we used the other points, it would be likely to oversample.
      double epsilonDistance = Math.sqrt(0.5) * heightMap.getGridResolutionXY();
      Point2DReadOnly gridCenter = heightMap.getGridCenter();
      int centerIndex = HeightMapTools.computeCenterIndex(heightMap.getGridSizeXY(), heightMap.getGridResolutionXY());
      int minIndexX = HeightMapTools.coordinateToIndex(polygonToSnap.getMinX(), gridCenter.getX(), heightMap.getGridResolutionXY(), centerIndex);
      int maxIndexX = HeightMapTools.coordinateToIndex(polygonToSnap.getMaxX(), gridCenter.getX(), heightMap.getGridResolutionXY(), centerIndex);
      int minIndexY = HeightMapTools.coordinateToIndex(polygonToSnap.getMinY(), gridCenter.getY(), heightMap.getGridResolutionXY(), centerIndex);
      int maxIndexY = HeightMapTools.coordinateToIndex(polygonToSnap.getMaxY(), gridCenter.getY(), heightMap.getGridResolutionXY(), centerIndex);

      Point3D maxPoint = new Point3D(0.0, 0.0, Double.NEGATIVE_INFINITY);

      for (int xIndex = minIndexX; xIndex <= maxIndexX; xIndex++)
      {
         for (int yIndex = minIndexY; yIndex <= maxIndexY; yIndex++)
         {
            double height = heightMap.getHeightAt(xIndex, yIndex);

            double x = HeightMapTools.indexToCoordinate(xIndex, gridCenter.getX(), heightMap.getGridResolutionXY(), centerIndex);
            double y = HeightMapTools.indexToCoordinate(yIndex, gridCenter.getY(), heightMap.getGridResolutionXY(), centerIndex);

            if (Double.isNaN(height) || height < minimumHeightToConsider)
            {
               continue;
            }

            Point3D point = new Point3D(x, y, height);
            double signedDistance = polygonToSnap.signedDistance(new Point2D(x, y));

            if (signedDistance > epsilonDistance)
            {
               continue;
            }

            if (height > maxPoint.getZ())
               maxPoint.set(point);

            heightMapPointsInPolygon.add(point);
         }
      }

      // Here, we're going to filter out the points that are below a certain distance. We want to increase this distance from MinZ so as to account for a
      // possible max slope.
      // FIXME It's worth noting that if a single point is significantly far enough above all the other points, this will remove the other points, making it an
      // FIXME invalid snap. That may or may not be what we actually want.
      double minZ = maxPoint.getZ() - snapHeightThreshold;
      double slope = Math.tan(minimumSurfaceInclineRadians);
      heightMapPointsInPolygon.removeIf(point ->
                                        {
                                           double distance = point.distanceXY(maxPoint);
                                           double extraHeight = distance * slope;
                                           return point.getZ() < minZ - extraHeight;
                                        });

      if (heightMapPointsInPolygon.isEmpty())
      {
         area = Double.NaN;
         return null;
      }

      // Here we want to collect all the points in the foot that are valid under the foothold, as an approximation of the foot area. This is much better than
      // trying to use other points, which don't provide a good estimate of the foot area.
      Point2DReadOnly corner0 = polygonToSnap.getVertex(0);
      Point2DReadOnly corner1 = polygonToSnap.getVertex(1);
      Point2DReadOnly corner2 = polygonToSnap.getVertex(2);
      Point2DReadOnly corner3 = polygonToSnap.getVertex(3);

      for (double edgeAlpha = 0.0; edgeAlpha <= 1.0; edgeAlpha += snapAreaResolution)
      {
         Point2D pointOnEdge1 = new Point2D();
         Point2D pointOnEdge2 = new Point2D();
         pointOnEdge1.interpolate(corner0, corner1, edgeAlpha);
         pointOnEdge2.interpolate(corner3, corner2, edgeAlpha);

         for (double interiorAlpha = 0.0; interiorAlpha <= 1.0; interiorAlpha += snapAreaResolution)
         {
            Point2D point = new Point2D();
            point.interpolate(pointOnEdge1, pointOnEdge2, interiorAlpha);

            double height = heightMap.getHeightAt(point.getX(), point.getY());

            double distance = point.distanceXY(maxPoint);
            double extraHeightFromSlope = slope * distance;
            double minHeight = maxPoint.getZ() - snapHeightThreshold - extraHeightFromSlope;
            minHeight = Math.max(minimumHeightToConsider, minHeight);

            if (Double.isNaN(height) || height < minHeight)
            {
               continue;
            }

            footPointsInEnvironment.add(new Point2D(point.getX(), point.getY()));
         }
      }

      if (footPointsInEnvironment.size() < 3)
      {
         area = Double.NaN;
         return null;
      }
      ConvexPolygon2D snappedPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInEnvironment));
      area = snappedPolygon.getArea();

      planeFitter.fitPlaneToPoints(heightMapPointsInPolygon, bestFitPlane);
      rootMeanSquaredError = 0.0;

      for (Point3DReadOnly heightMapPoint : heightMapPointsInPolygon)
      {
         double predictedHeight = bestFitPlane.getZOnPlane(heightMapPoint.getX(), heightMapPoint.getY());
         rootMeanSquaredError += MathTools.square(predictedHeight - heightMapPoint.getZ());
      }
      maxPossibleRMSError = MathTools.square(1.0 / snapAreaResolution) * MathTools.square(0.5 * snapHeightThreshold);

      if (bestFitPlane.containsNaN())
      {
         return null;
      }

      rootMeanSquaredError = Math.sqrt(rootMeanSquaredError / heightMapPointsInPolygon.size());

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

   public double getMaxPossibleRMSError()
   {
      return maxPossibleRMSError;
   }

   public double getNormalizedRMSError()
   {
      return rootMeanSquaredError / maxPossibleRMSError;
   }

   public double getArea()
   {
      return area;
   }
}
