package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class PlanarRegionSnapTools
{
   static boolean isPointInOtherRegion(Point2DReadOnly point, ConvexPolygon2DReadOnly regionToIgnore, List<ConvexPolygon2D> allRegions)
   {
      for (ConvexPolygon2D convexPolygon : allRegions)
      {
         if (regionToIgnore.equals(convexPolygon))
            continue;

         if (convexPolygon.isPointInside(point))
            return true;

         for (Point2DReadOnly vertex : convexPolygon.getVertexBufferView())
         {
            if (vertex.epsilonEquals(point, 1e-8))
               return true;
         }
      }

      return false;
   }

   static ConvexPolygon2DReadOnly getContainingConvexRegion(Point2DReadOnly pointToCheck, List<ConvexPolygon2D> convexPolygons)
   {
      int size = convexPolygons.size();
      for (int i = 0; i < size; i++)
      {
         ConvexPolygon2DReadOnly convexPolygon = convexPolygons.get(i);
         if (convexPolygon.isPointInside(pointToCheck))
            return convexPolygon;
      }

      return null;
   }

   public static PlanarRegion findHighestRegion(Point2DReadOnly point, List<PlanarRegion> planarRegionList, PlanarRegionConstraintDataParameters parameters)
   {
      return findHighestRegion(point.getX(), point.getY(), planarRegionList, parameters);
   }

   public static PlanarRegion findHighestRegion(double x, double y, List<PlanarRegion> planarRegionList, PlanarRegionConstraintDataParameters parameters)
   {
      ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
      tempPolygon.addVertex(0.5 * FootstepNode.gridSizeXY, 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(0.5 * FootstepNode.gridSizeXY, - 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(- 0.5 * FootstepNode.gridSizeXY, 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(- 0.5 * FootstepNode.gridSizeXY, - 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.update();
      tempPolygon.translate(x, y);

      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(tempPolygon, planarRegionList);
      if (intersectingRegions == null || intersectingRegions.isEmpty())
      {
         return null;
      }

      double highestPoint = Double.NEGATIVE_INFINITY;
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < intersectingRegions.size(); i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);
         double height = planarRegion.getPlaneZGivenXY(x, y);

         if (height > highestPoint)
         {
            highestPoint = height;
            highestPlanarRegion = planarRegion;
         }
      }

      return highestPlanarRegion;
   }

   public static PlanarRegion findHighestRegionWithProjection(Point2DReadOnly point, Vector2D projectionTranslationToPack,
                                                       PlanarRegionConstraintDataHolder constraintDataHolder, List<PlanarRegion> planarRegionList,
                                                       PlanarRegionConstraintDataParameters parameters)
   {
      return findHighestRegionWithProjection(point.getX(), point.getY(), projectionTranslationToPack, constraintDataHolder, planarRegionList, parameters);
   }

   public static PlanarRegion findHighestRegionWithProjection(double x, double y, Vector2D projectionTranslationToPack,
                                                       PlanarRegionConstraintDataHolder constraintDataHolder, List<PlanarRegion> planarRegionList,
                                                       PlanarRegionConstraintDataParameters parameters)
   {
      ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
      tempPolygon.addVertex(0.5 * FootstepNode.gridSizeXY, 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(0.5 * FootstepNode.gridSizeXY, - 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(- 0.5 * FootstepNode.gridSizeXY, 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(- 0.5 * FootstepNode.gridSizeXY, - 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.update();
      tempPolygon.translate(x, y);

      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(tempPolygon, planarRegionList);
      if (intersectingRegions == null || intersectingRegions.isEmpty())
      {
         return null;
      }

      double highestPoint = Double.NEGATIVE_INFINITY;
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < intersectingRegions.size(); i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);
         Vector3D projectionTranslation = projectPointIntoRegion(planarRegion, x, y, constraintDataHolder, parameters);
         double height;

         if(projectionTranslation.containsNaN())
         {
            // even if projection fails, remember highest region. this will be considered an obstacle
            height = planarRegion.getPlaneZGivenXY(x, y);
         }
         else
         {
            height = planarRegion.getPlaneZGivenXY(x + projectionTranslation.getX(), y + projectionTranslation.getY());
         }

         if (height > highestPoint)
         {
            highestPoint = height;
            highestPlanarRegion = planarRegion;
            projectionTranslationToPack.set(projectionTranslation);
         }
      }

      return highestPlanarRegion;
   }

   private static Vector3D projectPointIntoRegion(PlanarRegion region, double x, double y, PlanarRegionConstraintDataHolder constraintDataHolder,
                                           PlanarRegionConstraintDataParameters parameters)
   {
      Vector3D projectionTranslation = new Vector3D();
      Point3D pointToSnap = new Point3D();

      pointToSnap.set(x, y, region.getPlaneZGivenXY(x, y));
      region.transformFromWorldToLocal(pointToSnap);
      Point2D projectedPoint = new Point2D(pointToSnap);

      ConvexPolygon2DReadOnly scaledRegionPolygon = constraintDataHolder.getScaledRegionPolygon(region, projectedPoint, parameters);

      // scale didn't work
      if(scaledRegionPolygon == null)
      {
         projectionTranslation.setToNaN();
         return projectionTranslation;
      }

      double signedDistanceToPolygon = scaledRegionPolygon.signedDistance(projectedPoint);
      if(signedDistanceToPolygon <= 0.0)
      {
         // return, no need to project
         projectionTranslation.setToZero();
         return projectionTranslation;
      }

      if (parameters.enforceTranslationLessThanGridCell && signedDistanceToPolygon >= 0.5 * FootstepNode.gridSizeXY)
      {
         // Projection distance is too big. Must be smaller than half of the grid size
         projectionTranslation.setToNaN();
         return projectionTranslation;
      }

      boolean successfulProjection = scaledRegionPolygon.orthogonalProjection(projectedPoint);
      if(!successfulProjection)
      {
         projectionTranslation.setToNaN();
         return projectionTranslation;
      }

      projectionTranslation.set(projectedPoint.getX(), projectedPoint.getY(), 0.0);
      projectionTranslation.sub(pointToSnap.getX(), pointToSnap.getY(), 0.0);
      region.transformFromLocalToWorld(projectionTranslation);
      projectionTranslation.setZ(0.0);

      return projectionTranslation;
   }


   public static RigidBodyTransform getSnapTransformToRegion(Point2DReadOnly pointToSnap, PlanarRegion planarRegionToSnapTo)
   {
      Point3D point = new Point3D(pointToSnap);
      point.setZ(planarRegionToSnapTo.getPlaneZGivenXY(pointToSnap.getX(), pointToSnap.getY()));

      Vector3D surfaceNormal = new Vector3D();
      planarRegionToSnapTo.getNormal(surfaceNormal);

      RigidBodyTransform snapTransform = PlanarRegionSnapTools.createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
      PlanarRegionSnapTools.setTranslationSettingZAndPreservingXAndY(point, snapTransform);

      return snapTransform;
   }

   public static RigidBodyTransform createTransformToMatchSurfaceNormalPreserveX(Vector3D surfaceNormal)
   {
      Vector3D xAxis = new Vector3D();
      Vector3D yAxis = new Vector3D(0.0, 1.0, 0.0);

      xAxis.cross(yAxis, surfaceNormal);
      xAxis.normalize();
      yAxis.cross(surfaceNormal, xAxis);

      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.setColumns(xAxis, yAxis, surfaceNormal);
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      transformToReturn.setRotation(rotationMatrix);
      return transformToReturn;
   }

   public static void setTranslationSettingZAndPreservingXAndY(Point3DReadOnly point, RigidBodyTransform transformToReturn)
   {
      setTranslationSettingZAndPreservingXAndY(point.getX(), point.getY(), point.getX(), point.getY(), point.getZ(), transformToReturn);
   }

   public static void setTranslationSettingZAndPreservingXAndY(double x, double y, double xTranslated, double yTranslated, double z, RigidBodyTransform transformToReturn)
   {
      Vector3D newTranslation = new Vector3D(x, y, 0.0);
      transformToReturn.transform(newTranslation);
      newTranslation.scale(-1.0);
      newTranslation.add(xTranslated, yTranslated, z);

      transformToReturn.setTranslation(newTranslation);
   }
}
