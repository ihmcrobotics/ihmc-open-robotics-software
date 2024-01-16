package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.createTransformToMatchSurfaceNormalPreserveX;
import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.setTranslationSettingZAndPreservingXAndY;

public class HeightMapPolygonSnapper
{
   private final List<Point3D> pointsInsidePolygon = new ArrayList<>();
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
      pointsInsidePolygon.clear();
      bestFitPlane.setToNaN();

      // collect all the points in the foot that are valid under the foothold
      if (polygonToSnap.getNumberOfVertices() != 4)
         throw new RuntimeException("We aren't set up to use this");
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
            if (Double.isNaN(height) || height < minimumHeightToConsider)
            {
               continue;
            }

            pointsInsidePolygon.add(new Point3D(point.getX(), point.getY(), height));
         }
      }

      if (pointsInsidePolygon.isEmpty())
      {
         return null;
      }

      // FIXME It's worth noting that if a single point is significantly far enough above all the other points, this will remove the other points, making it an
      // FIXME invalid snap. That may or may not be what we actually want.
      double maxZ = pointsInsidePolygon.stream().mapToDouble(Point3D::getZ).max().getAsDouble();
      double minZ = maxZ - snapHeightThreshold;
      pointsInsidePolygon.removeIf(point -> point.getZ() < minZ);
      if (pointsInsidePolygon.size() < 3)
      {
         area = Double.NaN;
         return null;
      }
      ConvexPolygon2D snappedPolygon = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(pointsInsidePolygon));
      area = snappedPolygon.getArea();

      planeFitter.fitPlaneToPoints(pointsInsidePolygon, bestFitPlane);
      rootMeanSquaredError = 0.0;

      for (int i = 0; i < pointsInsidePolygon.size(); i++)
      {
         double predictedHeight = bestFitPlane.getZOnPlane(pointsInsidePolygon.get(i).getX(), pointsInsidePolygon.get(i).getY());
         rootMeanSquaredError += MathTools.square(predictedHeight - pointsInsidePolygon.get(i).getZ());
      }
      maxPossibleRMSError = MathTools.square(1.0 / snapAreaResolution) * MathTools.square(0.5 * snapHeightThreshold);

      if (bestFitPlane.containsNaN())
      {
         return null;
      }

      rootMeanSquaredError = Math.sqrt(rootMeanSquaredError / pointsInsidePolygon.size());

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
