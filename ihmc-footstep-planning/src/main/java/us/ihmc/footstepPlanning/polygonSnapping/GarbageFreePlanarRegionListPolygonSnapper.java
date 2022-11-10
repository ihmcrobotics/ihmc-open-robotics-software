package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper.heightEpsilonToTakeFlatterRegion;
import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper.minimumNormalZToConsider;

public class GarbageFreePlanarRegionListPolygonSnapper
{
   private final List<PlanarRegion> intersectingRegions = new ArrayList<>();

   private final GarbageFreePlanarRegionPolygonSnapper polygonSnapper = new GarbageFreePlanarRegionPolygonSnapper();

   private final RigidBodyTransform snapTransform = new RigidBodyTransform();
   private final RigidBodyTransform highestTransform = new RigidBodyTransform();
   private final Point3D highestVertexInWorld = new Point3D();

   public boolean snapPolygonToPlanarRegionsList(ConvexPolygon2DReadOnly polygonToSnap,
                                                 List<PlanarRegion> planarRegionsListToSnapTo,
                                                 double maximumRegionHeightToConsider,
                                                 PlanarRegion regionToPack,
                                                 RigidBodyTransform snapTransformToPack)
   {
      PlanarRegionTools.findPlanarRegionsIntersectingPolygon(polygonToSnap, planarRegionsListToSnapTo, intersectingRegions);

      if (intersectingRegions.isEmpty())
      {
         snapTransformToPack.setToNaN();
         return false;
      }

      return snapPolygonToRegionsUnderFoot(polygonToSnap, intersectingRegions, maximumRegionHeightToConsider, regionToPack, snapTransformToPack);
   }

   public boolean snapPolygonToRegionsUnderFoot(ConvexPolygon2DReadOnly polygonToSnap,
                                                 List<PlanarRegion> intersectingRegions,
                                                 double maximumRegionHeightToConsider,
                                                 PlanarRegion regionToPack,
                                                 RigidBodyTransform snapTransformToPack)
   {
      int numberOfIntersectingRegions = intersectingRegions.size();

      double highestZ = Double.NEGATIVE_INFINITY;
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < numberOfIntersectingRegions; i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);

         polygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegion, highestVertexInWorld, snapTransform);

         if (highestVertexInWorld.getZ() > maximumRegionHeightToConsider)
         {
            continue;
         }
         else if (highestVertexInWorld.getZ() > highestZ + heightEpsilonToTakeFlatterRegion)
         {
            highestZ = highestVertexInWorld.getZ();
            highestTransform.set(snapTransform);
            highestPlanarRegion = planarRegion;
         }
         else if (highestVertexInWorld.getZ() > highestZ - heightEpsilonToTakeFlatterRegion)
         {
            // Tie. Let's take the one with the flatter surface normal.

            if (Math.abs(planarRegion.getNormal().getZ()) > Math.abs(highestPlanarRegion.getNormal().getZ()))
            {
               highestZ = highestVertexInWorld.getZ();
               highestTransform.set(snapTransform);
               highestPlanarRegion = planarRegion;
            }
         }
      }

      if (highestPlanarRegion == null)
      {
         snapTransformToPack.setToNaN();
         return false;
      }

      if (Math.abs(highestPlanarRegion.getNormal().getZ()) < minimumNormalZToConsider)
      {
         snapTransformToPack.setToNaN();
         return false;
      }

      if (regionToPack != null)
      {
         regionToPack.set(highestPlanarRegion);
      }

      snapTransformToPack.set(highestTransform);
      return true;
   }
}
