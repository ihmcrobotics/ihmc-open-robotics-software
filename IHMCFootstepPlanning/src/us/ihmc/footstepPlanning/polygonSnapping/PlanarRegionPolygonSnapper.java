package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class PlanarRegionPolygonSnapper
{

   /**
    * Snaps an XY polygon down onto a PlanarRegion. Returns the RigidBodyTransform required to perform the snap.
    * @param polygonToSnap
    * @param planarRegionToSnapTo
    * @return RigidBodyTransform required to snap the polygon down onto the PlanarRegion
    */
   public static RigidBodyTransform snapPolygonToPlanarRegion(ConvexPolygon2d polygonToSnap, PlanarRegion planarRegionToSnapTo)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();

      return transformToReturn;
   }
}
