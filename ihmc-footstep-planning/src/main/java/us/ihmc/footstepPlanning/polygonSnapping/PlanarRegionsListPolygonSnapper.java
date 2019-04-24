package us.ihmc.footstepPlanning.polygonSnapping;

import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionsListPolygonSnapper
{

   /**
    * Snaps an XY polygon down onto a PlanarRegionsList. Returns the RigidBodyTransform required to perform the snap.
    *
    * @param polygonToSnap ConvexPolygon2d that is to be snapped to the PlanarRegionsList.
    * @param planarRegionsListToSnapTo PlanarRegionsList that the polygon will be snapped to.
    * @return RigidBodyTransform Transform required to snap the polygon down onto the PlanarRegion.
    */
   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2D polygonToSnap, PlanarRegionsList planarRegionsListToSnapTo)
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
   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2DReadOnly polygonToSnap, PlanarRegionsList planarRegionsListToSnapTo, PlanarRegion regionToPack)
   {
      return snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsListToSnapTo.getPlanarRegionsAsList(), regionToPack);
   }

   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2DReadOnly polygonToSnap, List<PlanarRegion> planarRegionsListToSnapTo, PlanarRegion regionToPack)
   {
      double allowableExtraZ = 0.003; // For close ones. When close, take one that is flatter...
      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(polygonToSnap, planarRegionsListToSnapTo);

      if ((intersectingRegions == null) || (intersectingRegions.isEmpty()))
      {
         return null;
      }

      int numberOfIntersectingRegions = intersectingRegions.size();

      Vector3D surfaceNormal = new Vector3D();
      Vector3D highestSurfaceNormal = new Vector3D();
      RigidBodyTransform highestTransform = null;
      double highestZ = Double.NEGATIVE_INFINITY;
      Point3D highestVertexInWorld = new Point3D();
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
