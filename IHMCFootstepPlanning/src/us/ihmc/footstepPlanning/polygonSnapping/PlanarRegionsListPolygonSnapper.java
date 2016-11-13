package us.ihmc.footstepPlanning.polygonSnapping;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class PlanarRegionsListPolygonSnapper
{

   /**
    * Snaps an XY polygon down onto a PlanarRegionsList. Returns the RigidBodyTransform required to perform the snap.
    *
    * @param polygonToSnap ConvexPolygon2d that is to be snapped to the PlanarRegionsList.
    * @param planarRegionsListToSnapTo PlanarRegionsList that the polygon will be snapped to.
    * @return RigidBodyTransform Transform required to snap the polygon down onto the PlanarRegion.
    */
   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2d polygonToSnap, PlanarRegionsList planarRegionsListToSnapTo)
   {
      return snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsListToSnapTo, null);
   }

   /**
    * Snaps an XY polygon down onto a PlanarRegionsList. Returns the RigidBodyTransform required to perform the snap.
    *
    * @param polygonToSnap ConvexPolygon2d that is to be snapped to the PlanarRegionsList.
    * @param planarRegionsListToSnapTo PlanarRegionsList that the polygon will be snapped to.
    * @param regionToPack the planar region that this snaps to will be packed here (can be null).
    * @return RigidBodyTransform Transform required to snap the polygon down onto the PlanarRegion.
    */
   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2d polygonToSnap, PlanarRegionsList planarRegionsListToSnapTo, PlanarRegion regionToPack)
   {
      double allowableExtraZ = 0.003; // For close ones. When close, take one that is flatter...
      List<PlanarRegion> intersectingRegions = planarRegionsListToSnapTo.findPlanarRegionsIntersectingPolygon(polygonToSnap);

      if ((intersectingRegions == null) || (intersectingRegions.isEmpty()))
      {
         return null;
      }

      int numberOfIntersectingRegions = intersectingRegions.size();

      Vector3d surfaceNormal = new Vector3d();
      Vector3d highestSurfaceNormal = new Vector3d();
      RigidBodyTransform highestTransform = null;
      double highestZ = Double.NEGATIVE_INFINITY;
      Point3d highestVertexInWorld = new Point3d();
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < numberOfIntersectingRegions; i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);

         RigidBodyTransform snapTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegion, highestVertexInWorld);

         if (highestVertexInWorld.getZ() > highestZ + allowableExtraZ)
         {
            highestZ = highestVertexInWorld.getZ();
            highestTransform = snapTransform;
            highestPlanarRegion = planarRegion;
         }

         else if (highestVertexInWorld.getZ() > highestZ - allowableExtraZ)
         {
            // Tie. Let's take the one with the flatter surface normal.
            planarRegion.getNormal(surfaceNormal);
            highestPlanarRegion.getNormal(highestSurfaceNormal);

            if (Math.abs(surfaceNormal.getZ()) > Math.abs(highestSurfaceNormal.getZ()))
            {
               highestZ = highestVertexInWorld.getZ();
               highestTransform = snapTransform;
               highestPlanarRegion = planarRegion;
            }
         }
      }

      //TODO: For now, just ignore Planar Regions that are tilted too much.
      //TODO: But later, we need to make sure they are not obstacles that need to be avoided...
      highestPlanarRegion.getNormal(highestSurfaceNormal);
      if (Math.abs(highestSurfaceNormal.getZ()) < 0.2)
         return null;

      if (regionToPack != null)
         regionToPack.set(highestPlanarRegion);
      return highestTransform;
   }
}
