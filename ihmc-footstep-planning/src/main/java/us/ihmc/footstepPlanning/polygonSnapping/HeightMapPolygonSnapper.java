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
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.EnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.perception.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;

public class HeightMapPolygonSnapper
{
   private final List<Point3DBasics> footPointsInEnvironment = new ArrayList<>();
   private final Plane3D bestFitPlane = new Plane3D();
   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();
   private final ConvexPolygon2D snappedPolygon = new ConvexPolygon2D();

   private double maxPossibleRMSError;
   private double rootMeanSquaredError;
   private double areaFraction;

   private double snapAreaResolution = 0.2;

   public void setSnapAreaResolution(double snapAreaResolution)
   {
      this.snapAreaResolution = snapAreaResolution;
   }

   public FootstepSnapData computeSnapData(DiscreteFootstep footstep,
                                           ConvexPolygon2DReadOnly polygonInStepFrame,
                                           EnvironmentHandler environmentHandler,
                                           double snapHeightThreshold,
                                           double minSurfaceIncline)
   {
      return computeSnapData(footstep.getX(),
                             footstep.getY(),
                             footstep.getYaw(),
                             polygonInStepFrame,
                             environmentHandler,
                             snapHeightThreshold,
                             minSurfaceIncline,
                             -Double.MAX_VALUE);
   }

   public FootstepSnapData computeSnapData(double stepX,
                                           double stepY,
                                           double stepYaw,
                                           ConvexPolygon2DReadOnly polygonInStepFrame,
                                           EnvironmentHandler environmentHandler,
                                           double snapHeightThreshold,
                                           double minSurfaceIncline)
   {
      return computeSnapData(stepX, stepY, stepYaw, polygonInStepFrame, environmentHandler, snapHeightThreshold, minSurfaceIncline, -Double.MAX_VALUE);
   }

   public FootstepSnapData computeSnapData(DiscreteFootstep footstep,
                                           ConvexPolygon2DReadOnly polygonInStepFrame,
                                           EnvironmentHandler environmentHandler,
                                           double snapHeightThreshold,
                                           double minSurfaceIncline,
                                           double minimumHeightToConsider)
   {
      return computeSnapData(footstep.getX(),
                             footstep.getY(),
                             footstep.getYaw(),
                             polygonInStepFrame,
                             environmentHandler,
                             snapHeightThreshold,
                             minSurfaceIncline,
                             minimumHeightToConsider);
   }

   public FootstepSnapData computeSnapData(double stepX,
                                           double stepY,
                                           double yaw,
                                           ConvexPolygon2DReadOnly polygonInStepFrame,
                                           EnvironmentHandler environmentHandler,
                                           double snapHeightThreshold,
                                           double minSurfaceIncline,
                                           double minimumHeightToConsider)
   {
      RigidBodyTransform footstepTransform = new RigidBodyTransform();
      DiscreteFootstepTools.getStepTransform(stepX, stepY, yaw, footstepTransform);

      ConvexPolygon2D footPolygonInWorld = new ConvexPolygon2D(polygonInStepFrame);
      footPolygonInWorld.applyTransform(footstepTransform);

      RigidBodyTransform snapTransform = snapPolygonToHeightMap(footPolygonInWorld,
                                                                environmentHandler,
                                                                snapHeightThreshold,
                                                                minSurfaceIncline,
                                                                minimumHeightToConsider);

      if (snapTransform == null)
      {
         return FootstepSnapData.emptyData();
      }
      else
      {
         FootstepSnapData snapData = new FootstepSnapData(snapTransform);

         // FIXME if using the terrain map, this will not have value.
         snapData.setRMSErrorHeightMap(rootMeanSquaredError / maxPossibleRMSError);
         snapData.setSnapAreaFraction(areaFraction);

         // get the cropped polygon back in sole frame.
         // FIXME if using the terrain map, this will not have value.
         snapData.getCroppedFoothold().set(snappedPolygon);
         snapData.getCroppedFoothold().applyInverseTransform(footstepTransform);

         return snapData;
      }
   }

   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap, EnvironmentHandler environmentHandler)
   {
      return snapPolygonToHeightMap(polygonToSnap, environmentHandler, Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE);
   }

   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap,
                                                    EnvironmentHandler environmentHandler,
                                                    double snapHeightThreshold,
                                                    double minSurfaceIncline)
   {
      return snapPolygonToHeightMap(polygonToSnap, environmentHandler, snapHeightThreshold, minSurfaceIncline, -Double.MAX_VALUE);
   }

   private RigidBodyTransform computeSnapFromTerrainMap(ConvexPolygon2DReadOnly polygonToSnap, TerrainMapData terrainMapData)
   {
      Point2DReadOnly centroid = polygonToSnap.getCentroid();
      int rIndex = terrainMapData.getLocalXIndex(centroid.getX());
      int cIndex = terrainMapData.getLocalYIndex(centroid.getY());

      double height = terrainMapData.getSnappedHeightLocal(rIndex, cIndex);
      UnitVector3DReadOnly normal = terrainMapData.getNormalLocal(rIndex, cIndex);

      // The surface normal must point up. If it does not, recreate it so that it does.
      if (normal.getZ() < 0.0)
      {
         UnitVector3D tempNormal = new UnitVector3D(normal);
         tempNormal.negate();
         normal = tempNormal;
      }

      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      PolygonSnapperTools.constructRotationToMatchSurfaceNormal(normal, transformToReturn.getRotation());
      PolygonSnapperTools.setTranslationToAchieveZAndPreserveXAndY(centroid, 0.0, height, transformToReturn.getRotation(), transformToReturn.getTranslation());

      // TODO need to compute the snapped polygon
      snappedPolygon.set(polygonToSnap);
      // TODO need to compute the RMS error on the GPU.
      rootMeanSquaredError = Double.NaN;
      areaFraction = terrainMapData.getSnappedAreaLocal(rIndex, cIndex);

      return transformToReturn;
   }

   private RigidBodyTransform computeSnapFromHeightMap(ConvexPolygon2DReadOnly polygonToSnap,
                                                       HeightMapData heightMapData,
                                                       TerrainMapData terrainMapData,
                                                       double snapHeightThreshold,
                                                       double minSurfaceIncline,
                                                       double minimumHeightToConsider)
   {

      if (!computeFootPointsInTheEnvironment(polygonToSnap, heightMapData, terrainMapData, snapHeightThreshold, minSurfaceIncline, minimumHeightToConsider, footPointsInEnvironment))
      {
         areaFraction = Double.NaN;
         return null;
      }

      bestFitPlane.setToNaN();

      snappedPolygon.set(Vertex3DSupplier.asVertex3DSupplier(footPointsInEnvironment));
      areaFraction = snappedPolygon.getArea() / polygonToSnap.getArea();

      planeFitter.fitPlaneToPoints(footPointsInEnvironment, bestFitPlane);
      rootMeanSquaredError = 0.0;

      for (Point3DReadOnly heightMapPoint : footPointsInEnvironment)
      {
         double predictedHeight = bestFitPlane.getZOnPlane(heightMapPoint.getX(), heightMapPoint.getY());
         rootMeanSquaredError += MathTools.square(predictedHeight - heightMapPoint.getZ());
      }
      maxPossibleRMSError = MathTools.square(1.0 / snapAreaResolution) * MathTools.square(0.5 * snapHeightThreshold);

      if (bestFitPlane.containsNaN())
      {
         return null;
      }

      rootMeanSquaredError = Math.sqrt(rootMeanSquaredError / footPointsInEnvironment.size());

      // get the point that is the center of the plane.
      Point2DReadOnly centroid = polygonToSnap.getCentroid();
      double height = bestFitPlane.getZOnPlane(centroid.getX(), centroid.getY());

      // compute the actual snap transform.
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      PolygonSnapperTools.constructRotationToMatchSurfaceNormal(bestFitPlane.getNormal(), transformToReturn.getRotation());
      PolygonSnapperTools.setTranslationToAchieveZAndPreserveXAndY(centroid, 0.0, height, transformToReturn.getRotation(), transformToReturn.getTranslation());

      return transformToReturn;
   }

   public boolean computeFootPointsInTheEnvironment(ConvexPolygon2DReadOnly polygonToSnap,
                                                    HeightMapData heightMapData,
                                                    TerrainMapData terrainMapData,
                                                    double snapHeightThreshold,
                                                    double minSurfaceIncline,
                                                    double minimumHeightToConsider,
                                                    List<Point3DBasics> footPointsInEnvironmentToPack)
   {
      footPointsInEnvironmentToPack.clear();

      if (polygonToSnap.getNumberOfVertices() != 4)
         throw new RuntimeException("We aren't set up to use this");

      // Here we want to collect all the points in the foot that are valid under the foothold, as an approximation of the foot area. This is much better than
      // trying to use other points straight from the height map, which don't provide a good estimate of the foot area. However, this can oversample some cells
      // on the height map, meaning the quality of the resulting slope may be decreased. Essentially, we are trading accuracy of area fit for accuracy of normal
      Point2DReadOnly corner0 = polygonToSnap.getVertex(0);
      Point2DReadOnly corner1 = polygonToSnap.getVertex(1);
      Point2DReadOnly corner2 = polygonToSnap.getVertex(2);
      Point2DReadOnly corner3 = polygonToSnap.getVertex(3);

      Point2D pointOnEdge1 = new Point2D();
      Point2D pointOnEdge2 = new Point2D();
      Point2D footPointToSnap = new Point2D();

      Point3D maxPoint = new Point3D(0.0, 0.0, Double.NEGATIVE_INFINITY);

      for (double edgeAlpha = 0.0; edgeAlpha <= 1.0; edgeAlpha += snapAreaResolution)
      {
         pointOnEdge1.interpolate(corner0, corner1, edgeAlpha);
         pointOnEdge2.interpolate(corner3, corner2, edgeAlpha);

         for (double interiorAlpha = 0.0; interiorAlpha <= 1.0; interiorAlpha += snapAreaResolution)
         {
            footPointToSnap.interpolate(pointOnEdge1, pointOnEdge2, interiorAlpha);

            double height;
            if (terrainMapData != null)
               height = terrainMapData.getHeightInWorld(footPointToSnap.getX(), footPointToSnap.getY());
            else
               height = heightMapData.getHeightAt(footPointToSnap.getX(), footPointToSnap.getY());

            if (Double.isNaN(height) || height < minimumHeightToConsider)
            {
               continue;
            }

            Point3D point = new Point3D(footPointToSnap.getX(), footPointToSnap.getY(), height);

            if (height > maxPoint.getZ())
               maxPoint.set(point);
            footPointsInEnvironmentToPack.add(point);
         }
      }

      double minZ = maxPoint.getZ() - snapHeightThreshold;
      double slope = Math.tan(minSurfaceIncline);
//      footPointsInEnvironmentToPack.removeIf(point ->
//                                       {
//                                          double distance = point.distanceXY(maxPoint);
//                                          double extraHeight = distance * slope;
//                                          return point.getZ() < minZ - extraHeight;
//                                       });

      return footPointsInEnvironmentToPack.size() >= 3;
   }

   /**
    * Snaps the given polygon to the height map by a least-squares plane fit.
    * <p>
    * - Any cells with heights below minimumHeightToConsider are ignored.
    * - Any cells with heights below maxZ - snapHeightThreshold are ignored, where maxZ is the max height within the polygon
    */
   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap,
                                                    EnvironmentHandler environmentHandler,
                                                    double snapHeightThreshold,
                                                    double minSurfaceIncline,
                                                    double minimumHeightToConsider)
   {
      RigidBodyTransform transformToReturn;

      TerrainMapData terrainMapData = environmentHandler.getTerrainMapData();
      HeightMapData heightMapData = environmentHandler.getHeightMapData();

      if (environmentHandler.hasTerrainMapData() && terrainMapData.hasSnapHeight() && terrainMapData.hasSnapNormal())
      {
         transformToReturn = computeSnapFromTerrainMap(polygonToSnap, terrainMapData);
      }
      else
      {
         transformToReturn = computeSnapFromHeightMap(polygonToSnap,
                                                      heightMapData,
                                                      terrainMapData,
                                                      snapHeightThreshold,
                                                      minSurfaceIncline,
                                                      minimumHeightToConsider);
      }

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

   public double getAreaFraction()
   {
      return areaFraction;
   }

   public ConvexPolygon2DReadOnly getSnappedPolygonInWorld()
   {
      return snappedPolygon;
   }
}
