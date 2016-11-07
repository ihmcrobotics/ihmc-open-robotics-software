package us.ihmc.footstepPlanning.polygonSnapping;

import javax.vecmath.Point2d;
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
   public static RigidBodyTransform snapPolygonToPlanarRegion(ConvexPolygon2d polygonToSnap, PlanarRegion planarRegionToSnapTo)
   {
      int numberOfVertices = polygonToSnap.getNumberOfVertices();

      double highestZ = Double.NEGATIVE_INFINITY;
      Point2d highestVertex = null;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = polygonToSnap.getVertex(i);
         double zWorld = planarRegionToSnapTo.getPlaneZGivenXY(vertex.getX(), vertex.getY());

         if (zWorld > highestZ)
         {
            highestZ = zWorld;
            highestVertex = vertex;
         }
      }
      if (highestVertex == null)
         return null;

      Vector3d surfaceNormal = new Vector3d();
      planarRegionToSnapTo.getNormal(surfaceNormal);

      RigidBodyTransform transformToReturn = createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
      setTranslationSettingZAndPreservingXAndY(highestZ, highestVertex, transformToReturn);

      return transformToReturn;
   }

   private static void setTranslationSettingZAndPreservingXAndY(double highestZ, Point2d highestVertex, RigidBodyTransform transformToReturn)
   {
      Vector3d newTranslation = new Vector3d(highestVertex.getX(), highestVertex.getY(), 0.0);
      transformToReturn.transform(newTranslation);
      newTranslation.scale(-1.0);
      newTranslation.add(new Vector3d(highestVertex.getX(), highestVertex.getY(), highestZ));

      transformToReturn.setTranslation(newTranslation);
   }

   private static RigidBodyTransform createTransformToMatchSurfaceNormalPreserveX(Vector3d surfaceNormal)
   {
      Vector3d xAxis = new Vector3d(1.0, 0.0, 0.0);
      Vector3d yAxis = new Vector3d();

      yAxis.cross(surfaceNormal, xAxis);
      yAxis.normalize();
      xAxis.cross(yAxis, surfaceNormal);

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
