package us.ihmc.rdx.mesh;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;

public class MeshDataGeneratorMissing
{
   private static final float TwoPi = 2.0f * (float) Math.PI;
   private static final float HalfPi = (float) Math.PI / 2.0f;

   public static MeshDataHolder InvertedSphere(double radius, int latitudeN, int longitudeN)
   {
      return InvertedEllipsoid((float) radius, (float) radius, (float) radius, latitudeN, longitudeN);
   }

   public static MeshDataHolder InvertedEllipsoid(float xRadius, float yRadius, float zRadius, int latitudeN, int longitudeN)
   {
      // Reminder of longitude and latitude: http://www.geographyalltheway.com/ks3_geography/maps_atlases/longitude_latitude.htm
      Point3D32 points[] = new Point3D32[(latitudeN - 1) * longitudeN + 2];
      Vector3D32[] normals = new Vector3D32[(latitudeN - 1) * longitudeN + 2];
      TexCoord2f textPoints[] = new TexCoord2f[(latitudeN - 1) * longitudeN + 2];

      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         float longitudeAngle = TwoPi * ((float) longitudeIndex / (float) longitudeN);
         float cosLongitude = (float) Math.cos(longitudeAngle);
         float sinLongitude = (float) Math.sin(longitudeAngle);

         for (int latitudeIndex = 1; latitudeIndex < latitudeN; latitudeIndex++)
         {
            float latitudeAngle = (float) (-HalfPi + Math.PI * ((float) latitudeIndex / (float) latitudeN));
            float cosLatitude = (float) Math.cos(latitudeAngle);
            float sinLatitude = (float) Math.sin(latitudeAngle);

            int currentIndex = (latitudeIndex - 1) * longitudeN + longitudeIndex;
            float normalX = cosLongitude * cosLatitude;
            float normalY = sinLongitude * cosLatitude;
            float normalZ = sinLatitude;
            float vertexX = xRadius * normalX;
            float vertexY = yRadius * normalY;
            float vertexZ = zRadius * normalZ;
            points[currentIndex] = new Point3D32(vertexX, vertexY, vertexZ);

            normals[currentIndex] = new Vector3D32(normalX, normalY, normalZ);

            float textureX = longitudeAngle / TwoPi;
            float textureY = (float) (0.5 * sinLatitude + 0.5);
            textPoints[currentIndex] = new TexCoord2f(textureX, textureY);
         }
      }

      // South pole
      int southPoleIndex = (latitudeN - 1) * longitudeN;
      points[southPoleIndex] = new Point3D32(0.0f, 0.0f, -zRadius);
      normals[southPoleIndex] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[southPoleIndex] = new TexCoord2f(0.5f, 0.0f);

      // North pole
      int northPoleIndex = (latitudeN - 1) * longitudeN + 1;
      points[northPoleIndex] = new Point3D32(0.0f, 0.0f, zRadius);
      normals[northPoleIndex] = new Vector3D32(0.0f, 0.0f, 1.0f);
      textPoints[northPoleIndex] = new TexCoord2f(1.0f, 1.0f);

      int numberOfTriangles = 2 * (latitudeN - 1) * longitudeN + 2 * longitudeN;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Mid-latitude faces
      for (int latitudeIndex = 0; latitudeIndex < latitudeN - 2; latitudeIndex++)
      {
         for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
         {
            int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
            int nextLatitudeIndex = latitudeIndex + 1;

            // Lower triangles
            triangleIndices[index++] = latitudeIndex * longitudeN + longitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + longitudeIndex;
            triangleIndices[index++] = latitudeIndex * longitudeN + nextLongitudeIndex;
            // Upper triangles
            triangleIndices[index++] = latitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + longitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + nextLongitudeIndex;
         }
      }

      // South pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
         triangleIndices[index++] = southPoleIndex;
         triangleIndices[index++] = longitudeIndex;
         triangleIndices[index++] = nextLongitudeIndex;
      }

      // North pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
         triangleIndices[index++] = northPoleIndex;
         triangleIndices[index++] = (latitudeN - 2) * longitudeN + nextLongitudeIndex;
         triangleIndices[index++] = (latitudeN - 2) * longitudeN + longitudeIndex;
      }

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
   }
}
