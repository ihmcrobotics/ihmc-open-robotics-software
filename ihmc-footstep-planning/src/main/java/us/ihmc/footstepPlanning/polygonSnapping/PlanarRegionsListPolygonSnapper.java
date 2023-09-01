package us.ihmc.footstepPlanning.polygonSnapping;

import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionsListPolygonSnapper
{
   /** If two regions have snaps within this height threshold, take the flatter one */
   static final double heightEpsilonToTakeFlatterRegion = 0.003;

   /** Return null snap if region has this (or higher) incline. 0 if flat and half-pi is vertical surface */
   static final double maximumInclineToConsider = Math.toRadians(75.0);
   static final double minimumNormalZToConsider = Math.cos(maximumInclineToConsider);

   /**
    * Snaps an XY polygon down onto a PlanarRegionsList. Returns the RigidBodyTransform required to perform the snap.
    *
    * @param polygonToSnap ConvexPolygon2d that is to be snapped to the PlanarRegionsList.
    * @param planarRegionsListToSnapTo PlanarRegionsList that the polygon will be snapped to.
    * @return RigidBodyTransform Transform required to snap the polygon down onto the PlanarRegion.
    */
   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2D polygonToSnap, PlanarRegionsList planarRegionsListToSnapTo, double maximumRegionHeightToConsider)
   {
      return snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsListToSnapTo, maximumRegionHeightToConsider, null);
   }

   /**
    * Snaps an XY polygon down onto a PlanarRegionsList. Returns the RigidBodyTransform required to perform the snap.
    *
    * @param polygonToSnap ConvexPolygon2d that is to be snapped to the PlanarRegionsList.
    * @param planarRegionsListToSnapTo PlanarRegionsList that the polygon will be snapped to.
    * @param regionToPack the planar region that this snaps to will be packed here (can be null).
    * @return RigidBodyTransform Transform required to snap the polygon down onto the PlanarRegion.
    */
   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2DReadOnly polygonToSnap, PlanarRegionsList planarRegionsListToSnapTo, double maximumRegionHeightToConsider, PlanarRegion regionToPack)
   {
      return snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsListToSnapTo.getPlanarRegionsAsList(), maximumRegionHeightToConsider, regionToPack);
   }

   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2DReadOnly polygonToSnap, List<PlanarRegion> planarRegionsListToSnapTo, double maximumRegionHeightToConsider, PlanarRegion regionToPack)
   {
      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(polygonToSnap, planarRegionsListToSnapTo);

      if ((intersectingRegions == null) || (intersectingRegions.isEmpty()))
      {
         return null;
      }

      int numberOfIntersectingRegions = intersectingRegions.size();

      RigidBodyTransform highestTransform = null;
      double highestZ = Double.NEGATIVE_INFINITY;
      Point3D highestVertexInWorld = new Point3D();
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < numberOfIntersectingRegions; i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);

         RigidBodyTransform snapTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegion, highestVertexInWorld);

         if (highestVertexInWorld.getZ() > maximumRegionHeightToConsider)
         {
            continue;
         }
         else if (highestVertexInWorld.getZ() > highestZ + heightEpsilonToTakeFlatterRegion)
         {
            highestZ = highestVertexInWorld.getZ();
            highestTransform = snapTransform;
            highestPlanarRegion = planarRegion;
         }
         else if (highestVertexInWorld.getZ() > highestZ - heightEpsilonToTakeFlatterRegion)
         {
            // Tie. Let's take the one with the flatter surface normal.

            if (Math.abs(planarRegion.getNormal().getZ()) > Math.abs(highestPlanarRegion.getNormal().getZ()))
            {
               highestZ = highestVertexInWorld.getZ();
               highestTransform = snapTransform;
               highestPlanarRegion = planarRegion;
            }
         }
      }

      if (highestPlanarRegion == null)
      {
         return null;
      }

      if (Math.abs(highestPlanarRegion.getNormal().getZ()) < minimumNormalZToConsider)
      {
         return null;
      }

      if (regionToPack != null)
      {
         regionToPack.set(highestPlanarRegion);
      }

      return highestTransform;
   }
}
