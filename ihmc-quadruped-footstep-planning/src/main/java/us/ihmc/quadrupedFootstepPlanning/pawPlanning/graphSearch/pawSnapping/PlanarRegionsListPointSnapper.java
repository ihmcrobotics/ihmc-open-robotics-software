package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

public class PlanarRegionsListPointSnapper
{
   public static RigidBodyTransform snapPointToPlanarRegionsList(Point2DReadOnly pointInWorldToSnap, List<PlanarRegion> planarRegionsListToSnapTo,
                                                                 PlanarRegion regionToPack)
   {
      double allowableExtraZ = 0.003; // For close ones. When close, take one that is flatter...
      List<PlanarRegion> intersectingRegions = PlanarRegionTools
            .findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegionsListToSnapTo, pointInWorldToSnap);

      if ((intersectingRegions == null) || (intersectingRegions.isEmpty()))
      {
         return null;
      }

      int numberOfIntersectingRegions = intersectingRegions.size();

      Vector3D surfaceNormal = new Vector3D();
      Vector3D highestSurfaceNormal = new Vector3D();
      double highestZ = Double.NEGATIVE_INFINITY;
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < numberOfIntersectingRegions; i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);

         double regionHeight = intersectingRegions.get(i).getPlaneZGivenXY(pointInWorldToSnap.getX(), pointInWorldToSnap.getY());

         if (regionHeight > highestZ + allowableExtraZ)
         {
            highestZ = regionHeight;
            highestPlanarRegion = planarRegion;
         }
         else if (regionHeight > highestZ - allowableExtraZ)
         {
            // Tie. Let's take the one with the flatter surface normal.
            planarRegion.getNormal(surfaceNormal);
            highestPlanarRegion.getNormal(highestSurfaceNormal);

            if (Math.abs(surfaceNormal.getZ()) > Math.abs(highestSurfaceNormal.getZ()))
            {
               highestZ = regionHeight;
               highestPlanarRegion = planarRegion;
            }
         }
      }


      RigidBodyTransform highestPlanarRegionTransform = new RigidBodyTransform();

      highestPlanarRegion.getNormal(highestSurfaceNormal);
      highestPlanarRegion.getTransformToWorld(highestPlanarRegionTransform);

      Point3D highestIntersectionPointInWorldFrame = new Point3D(pointInWorldToSnap.getX(), pointInWorldToSnap.getY(), 0.0);
      highestPlanarRegionTransform.transform(highestIntersectionPointInWorldFrame);

      RigidBodyTransform highestTransform = createTransformToMatchSurfaceNormalPreserveX(highestSurfaceNormal);
      setTranslationSettingZAndPreservingXAndY(highestIntersectionPointInWorldFrame, highestTransform);

      //TODO: For now, just ignore Planar Regions that are tilted too much.
      //TODO: But later, we need to make sure they are not obstacles that need to be avoided...
      if (Math.abs(highestSurfaceNormal.getZ()) < 0.2)
         return null;

      if (regionToPack != null)
         regionToPack.set(highestPlanarRegion);

      return highestTransform;
   }

   private static void setTranslationSettingZAndPreservingXAndY(Point3DReadOnly point, RigidBodyTransform transformToReturn)
   {
      Vector3D newTranslation = new Vector3D(point.getX(), point.getY(), 0.0);
      transformToReturn.transform(newTranslation);
      newTranslation.scale(-1.0);
      newTranslation.add(point);

      transformToReturn.setTranslation(newTranslation);
   }

   private static RigidBodyTransform createTransformToMatchSurfaceNormalPreserveX(Vector3D surfaceNormal)
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
}
