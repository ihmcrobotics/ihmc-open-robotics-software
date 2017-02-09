package us.ihmc.footstepPlanning.polygonSnapping;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

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
   public static RigidBodyTransform snapPolygonToPlanarRegion(ConvexPolygon2d polygonToSnap, PlanarRegion planarRegionToSnapTo, Point3d highestVertexInWorld)
   {
      ArrayList<ConvexPolygon2d> polygonIntersections = new ArrayList<>();
      planarRegionToSnapTo.getPolygonIntersectionsWhenProjectedVertically(polygonToSnap, polygonIntersections);

      if (polygonIntersections.isEmpty())
         return null;

      RigidBodyTransform transform = new RigidBodyTransform();
      planarRegionToSnapTo.getTransformToWorld(transform);
      Point3d vertexInWorld = new Point3d();

      int numberOfIntersectingPolygons = polygonIntersections.size();
      double highestZ = Double.NEGATIVE_INFINITY;
      Point2d highestIntersectionVertexInPlaneFrame = null;

      for (int i = 0; i < numberOfIntersectingPolygons; i++)
      {
         ConvexPolygon2d intersectingPolygon = polygonIntersections.get(i);

         int numberOfVertices = intersectingPolygon.getNumberOfVertices();

         for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
         {
            Point2d vertex = intersectingPolygon.getVertex(vertexIndex);
            vertexInWorld.set(vertex.getX(), vertex.getY(), 0.0);
            transform.transform(vertexInWorld);

            if (vertexInWorld.getZ() > highestZ)
            {
               highestZ = vertexInWorld.getZ();
               highestIntersectionVertexInPlaneFrame = vertex;
               highestVertexInWorld.set(vertexInWorld);
            }
         }
      }

      if (highestIntersectionVertexInPlaneFrame == null)
         return null;

      Vector3d surfaceNormal = new Vector3d();
      planarRegionToSnapTo.getNormal(surfaceNormal);

      Point3d highestIntersectionVertexInWorldFrame = new Point3d(highestIntersectionVertexInPlaneFrame.getX(), highestIntersectionVertexInPlaneFrame.getY(), 0.0);
      transform.transform(highestIntersectionVertexInWorldFrame);
      highestIntersectionVertexInPlaneFrame.set(highestIntersectionVertexInWorldFrame.getX(), highestIntersectionVertexInWorldFrame.getY());

      RigidBodyTransform transformToReturn = createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
      setTranslationSettingZAndPreservingXAndY(highestIntersectionVertexInWorldFrame, transformToReturn);

      return transformToReturn;
   }

   private static void setTranslationSettingZAndPreservingXAndY(Point3d highestVertex, RigidBodyTransform transformToReturn)
   {
      Vector3d newTranslation = new Vector3d(highestVertex.getX(), highestVertex.getY(), 0.0);
      transformToReturn.transform(newTranslation);
      newTranslation.scale(-1.0);
      newTranslation.add(highestVertex);

      transformToReturn.setTranslation(newTranslation);
   }

   private static RigidBodyTransform createTransformToMatchSurfaceNormalPreserveX(Vector3d surfaceNormal)
   {
      Vector3d xAxis = new Vector3d();
      Vector3d yAxis = new Vector3d(0.0, 1.0, 0.0);

      xAxis.cross(yAxis, surfaceNormal);
      xAxis.normalize();
      yAxis.cross(surfaceNormal, xAxis);

      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      transformToReturn.setM00(xAxis.getX());
      transformToReturn.setM10(xAxis.getY());
      transformToReturn.setM20(xAxis.getZ());

      transformToReturn.setM01(yAxis.getX());
      transformToReturn.setM11(yAxis.getY());
      transformToReturn.setM21(yAxis.getZ());

      transformToReturn.setM02(surfaceNormal.getX());
      transformToReturn.setM12(surfaceNormal.getY());
      transformToReturn.setM22(surfaceNormal.getZ());
      return transformToReturn;
   }
}
