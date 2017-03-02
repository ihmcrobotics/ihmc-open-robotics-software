package us.ihmc.footstepPlanning.polygonSnapping;

import java.util.ArrayList;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionPolygonSnapper
{
   /**
    * Snaps an XY polygon down onto a PlanarRegion. Returns the RigidBodyTransform required to perform the snap.
    * @param polygonToSnap
    * @param planarRegionToSnapTo
    * @return RigidBodyTransform required to snap the polygon down onto the PlanarRegion
    */
   public static RigidBodyTransform snapPolygonToPlanarRegion(ConvexPolygon2d polygonToSnap, PlanarRegion planarRegionToSnapTo, Point3D highestVertexInWorld)
   {
      ArrayList<ConvexPolygon2d> polygonIntersections = new ArrayList<>();
      planarRegionToSnapTo.getPolygonIntersectionsWhenProjectedVertically(polygonToSnap, polygonIntersections);

      if (polygonIntersections.isEmpty())
         return null;

      RigidBodyTransform transform = new RigidBodyTransform();
      planarRegionToSnapTo.getTransformToWorld(transform);
      Point3D vertexInWorld = new Point3D();

      int numberOfIntersectingPolygons = polygonIntersections.size();
      double highestZ = Double.NEGATIVE_INFINITY;
      Point2D highestIntersectionVertexInPlaneFrame = null;

      for (int i = 0; i < numberOfIntersectingPolygons; i++)
      {
         ConvexPolygon2d intersectingPolygon = polygonIntersections.get(i);

         int numberOfVertices = intersectingPolygon.getNumberOfVertices();

         for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
         {
            Point2DReadOnly vertex = intersectingPolygon.getVertex(vertexIndex);
            vertexInWorld.set(vertex.getX(), vertex.getY(), 0.0);
            transform.transform(vertexInWorld);

            if (vertexInWorld.getZ() > highestZ)
            {
               highestZ = vertexInWorld.getZ();
               highestIntersectionVertexInPlaneFrame = new Point2D(vertex);
               highestVertexInWorld.set(vertexInWorld);
            }
         }
      }

      if (highestIntersectionVertexInPlaneFrame == null)
         return null;

      Vector3D surfaceNormal = new Vector3D();
      planarRegionToSnapTo.getNormal(surfaceNormal);

      Point3D highestIntersectionVertexInWorldFrame = new Point3D(highestIntersectionVertexInPlaneFrame.getX(), highestIntersectionVertexInPlaneFrame.getY(), 0.0);
      transform.transform(highestIntersectionVertexInWorldFrame);
      highestIntersectionVertexInPlaneFrame.set(highestIntersectionVertexInWorldFrame.getX(), highestIntersectionVertexInWorldFrame.getY());

      RigidBodyTransform transformToReturn = createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
      setTranslationSettingZAndPreservingXAndY(highestIntersectionVertexInWorldFrame, transformToReturn);

      return transformToReturn;
   }

   private static void setTranslationSettingZAndPreservingXAndY(Point3D highestVertex, RigidBodyTransform transformToReturn)
   {
      Vector3D newTranslation = new Vector3D(highestVertex.getX(), highestVertex.getY(), 0.0);
      transformToReturn.transform(newTranslation);
      newTranslation.scale(-1.0);
      newTranslation.add(highestVertex);

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
