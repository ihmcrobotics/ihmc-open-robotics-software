package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;

import java.util.List;

public class PlanarRegionPolygonSnapper
{
   /**
    * Snaps an XY polygon down onto a PlanarRegion. Returns the RigidBodyTransform required to perform the snap.
    * <p>
    * After this snap is performed, the only point on the polygon that maintains its original X-Y location is the highest point. The other points will likely
    * be translated, which is required if the planar region is not flat.
    * </p>
    * <p>
    * If the snap is unsuccessful, this will return a null transform. This would be unsuccessful if there is no intersection between the polygon and the planar
    * region.
    * </p>
    *
    * @param polygonToSnap              polygon to snap down. Not modified.
    * @param planarRegionToSnapTo       region to snap the polygon to. Not modified.
    * @param highestVertexInWorldToPack highest vertex of the planar region that has been snapped down. Modified.
    * @return RigidBodyTransform required to snap the polygon down onto the PlanarRegion. Null if snap is unsuccessful.
    */
   public static RigidBodyTransform snapPolygonToPlanarRegion(ConvexPolygon2DReadOnly polygonToSnap,
                                                              PlanarRegion planarRegionToSnapTo,
                                                              Point3DBasics highestVertexInWorldToPack)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      if (snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorldToPack, transformToReturn))
         return transformToReturn;

      return null;
   }

   /**
    * Snaps an XY polygon down onto a PlanarRegion. Returns the RigidBodyTransform required to perform the snap.
    * <p>
    * After this snap is performed, the only point on the polygon that maintains its original X-Y location is the highest point. The other points will likely
    * be translated, which is required if the planar region is not flat.
    * </p>
    * <p>
    * If the snap is unsuccessful, this will return a false. This would be unsuccessful if there is no intersection between the polygon and the planar region.
    * </p>
    *
    * @param polygonToSnap              polygon to snap down. Not modified.
    * @param planarRegionToSnapTo       region to snap the polygon to. Not modified.
    * @param highestVertexInWorldToPack highest vertex of the planar region that has been snapped down. Modified.
    * @param snapTransformToPack        transform applied to the polygon to snap it onto the planar region. Modified.
    * @return whether the snap was successful.
    */
   public static boolean snapPolygonToPlanarRegion(ConvexPolygon2DReadOnly polygonToSnap,
                                                   PlanarRegion planarRegionToSnapTo,
                                                   Point3DBasics highestVertexInWorldToPack,
                                                   RigidBodyTransform snapTransformToPack)
   {
      List<Point2DReadOnly> polygonIntersections = PlanarRegionTools.getPolygonIntersectionsWhenProjectedVertically(planarRegionToSnapTo, polygonToSnap);

      if (polygonIntersections.isEmpty())
         return false;

      Point3D vertexInWorld = new Point3D();

      int numberOfIntersecingPoints = polygonIntersections.size();
      double highestZ = Double.NEGATIVE_INFINITY;
      boolean noIntersection = true;

      for (int i = 0; i < numberOfIntersecingPoints; i++)
      {
         Point2DReadOnly vertex = polygonIntersections.get(i);
         vertexInWorld.set(vertex.getX(), vertex.getY(), 0.0);
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
