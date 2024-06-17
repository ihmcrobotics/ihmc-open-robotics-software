package us.ihmc.footstepPlanning.polygonSnapping;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;

public class PlanarRegionPolygonSnapper
{
   /**
    * Snaps an XY polygon down onto a PlanarRegion. Returns the RigidBodyTransform required to perform the snap.
    * @param polygonToSnap
    * @param planarRegionToSnapTo
    * @return RigidBodyTransform required to snap the polygon down onto the PlanarRegion
    */
   public static RigidBodyTransform snapPolygonToPlanarRegion(ConvexPolygon2DReadOnly polygonToSnap, PlanarRegion planarRegionToSnapTo, Point3D highestVertexInWorld)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      if (snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld, transformToReturn))
         return transformToReturn;

      return null;
   }

   /**
    * Snaps an XY polygon down onto a PlanarRegion. Returns the RigidBodyTransform required to perform the snap.
    * @param polygonToSnap
    * @param planarRegionToSnapTo
    * @return RigidBodyTransform required to snap the polygon down onto the PlanarRegion
    */
   public static boolean snapPolygonToPlanarRegion(ConvexPolygon2DReadOnly polygonToSnap,
                                                   PlanarRegion planarRegionToSnapTo,
                                                   Point3D highestVertexInWorld,
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
            highestVertexInWorld.set(vertexInWorld);
         }
      }

      if (noIntersection)
         return false;

      PolygonSnapperTools.constructTransformToMatchSurfaceNormalPreserveX(planarRegionToSnapTo.getNormal(), snapTransformToPack);
      setTranslationSettingZAndPreservingXAndY(highestVertexInWorld, snapTransformToPack);

      return true;
   }

   static void setTranslationSettingZAndPreservingXAndY(Point3DReadOnly highestVertex, RigidBodyTransform transformToReturn)
   {
      EuclidCoreMissingTools.transform(transformToReturn.getRotation(), highestVertex.getX(), highestVertex.getY(), 0.0, transformToReturn.getTranslation());
      transformToReturn.getTranslation().scale(-1.0);
      transformToReturn.getTranslation().add(highestVertex);
   }
}
