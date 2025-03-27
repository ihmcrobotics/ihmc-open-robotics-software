package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;

public class GarbageFreePlanarRegionPolygonSnapper
{
   private final RecyclingArrayList<Point2DBasics> intersectionPoints = new RecyclingArrayList<>(Point2D::new);
   private final PlanarRegionTools planarRegionTools = new PlanarRegionTools();

   private final Point3D vertexInWorld = new Point3D();

   /**
    * Snaps an XY polygon down onto a PlanarRegion. Returns the RigidBodyTransform required to perform the snap.
    * @param polygonToSnap
    * @param planarRegionToSnapTo
    * @return RigidBodyTransform required to snap the polygon down onto the PlanarRegion
    */
   public boolean snapPolygonToPlanarRegion(ConvexPolygon2DReadOnly polygonToSnap,
                                            PlanarRegion planarRegionToSnapTo,
                                            Point3DBasics highestVertexInWorldToPack,
                                            RigidBodyTransform snapTransformToPack)
   {
      planarRegionTools.getPolygonIntersectionsWhenProjectedVertically(planarRegionToSnapTo, polygonToSnap, intersectionPoints);

      if (intersectionPoints.isEmpty())
         return false;

      highestVertexInWorldToPack.setToNaN();

      int numberOfIntersecingPoints = intersectionPoints.size();
      double highestZ = Double.NEGATIVE_INFINITY;
      boolean noIntersection = true;

      for (int i = 0; i < numberOfIntersecingPoints; i++)
      {
         Point2DReadOnly vertex = intersectionPoints.get(i);
         vertexInWorld.set(vertex, 0.0);
         planarRegionToSnapTo.getTransformToWorld().transform(vertexInWorld);

         if (vertexInWorld.getZ() > highestZ)
         {
            highestZ = vertexInWorld.getZ();
            noIntersection = false;
            highestVertexInWorldToPack.set(vertexInWorld);
         }
      }

      if (noIntersection)
         return false;

      PolygonSnapperTools.constructRotationToMatchSurfaceNormal(planarRegionToSnapTo.getNormal(), snapTransformToPack.getRotation());
      PolygonSnapperTools.setTranslationSettingZAndPreservingXAndY(highestVertexInWorldToPack, snapTransformToPack);

      return true;
   }

}
