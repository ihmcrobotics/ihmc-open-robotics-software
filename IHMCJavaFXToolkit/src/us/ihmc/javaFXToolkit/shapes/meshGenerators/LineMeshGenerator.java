package us.ihmc.javaFXToolkit.shapes.meshGenerators;

import java.util.Arrays;

import javax.vecmath.Vector3f;

public class LineMeshGenerator
{
   public static FXMeshDataHolder lineMesh(double lx, double ly, double lz, double width)
   {
      return lineMesh((float) lx, (float) ly, (float) lz, (float) width); 
   }

   public static FXMeshDataHolder lineMesh(float lx, float ly, float lz, float width)
   {
      FXMeshDataHolder lineMesh = new FXMeshDataHolder();
      lineMesh.setVertexCoordinates(generatePoints(lx, ly, lz, width));
      lineMesh.setTextureCoordinates(Arrays.copyOf(texCoords, texCoords.length));
      lineMesh.setFaceIndices(Arrays.copyOf(faces, faces.length));
      lineMesh.setFaceSmoothingGroups(Arrays.copyOf(faceSmoothingGroups, faceSmoothingGroups.length));
      return lineMesh;
   }

   public static float[] generatePoints(float lx, float ly, float lz, float width)
   {
      Vector3f localDirection = new Vector3f(lx, ly, lz);
      float lineLength = localDirection.length();
      localDirection.scale(1.0f / lineLength);

      float halfL = lineLength / 2.0f;

      float points[] = BoxMeshGenerator.generatePoints(width, width, lineLength);

      for (int i = 2; i < points.length; i += 3)
         points[i] += halfL;

      float yaw;
      float pitch;
      if (Math.abs(localDirection.getZ()) < 1.0 - 1.0e-3)
      {
         yaw = (float) Math.atan2(localDirection.getY(), localDirection.getX());
         double xyLength = Math.sqrt(localDirection.getX() * localDirection.getX() + localDirection.getY() * localDirection.getY());
         pitch = (float) Math.atan2(xyLength, localDirection.getZ());
      }
      else
      {
         yaw = 0.0f;
         pitch = localDirection.getZ() >= 0.0 ? 0.0f : (float) Math.PI;
      }

      float cYaw = (float) Math.cos(yaw);
      float sYaw = (float) Math.sin(yaw);

      float cPitch = (float) Math.cos(pitch);
      float sPitch = (float) Math.sin(pitch);

      float rxx = cYaw * cPitch;
      float rxy = - sYaw;
      float rxz = cYaw * sPitch;
      float ryx = sYaw * cPitch;
      float ryy = cYaw;
      float ryz = sYaw * sPitch;
      float rzx = -sPitch;
      float rzz = cPitch;

      for (int i = 0; i < points.length; i += 3)
      {
         float x = points[i];
         float y = points[i + 1];
         float z = points[i + 2];
         points[i    ] = rxx * x + rxy * y + rxz * z;
         points[i + 1] = ryx * x + ryy * y + ryz * z;
         points[i + 2] = rzx * x           + rzz * z;
      }

      return points;
   }

   public static final float texCoords[] = BoxMeshGenerator.texCoords;

   public static final int faceSmoothingGroups[] = BoxMeshGenerator.faceSmoothingGroups;

   public static final int faces[] = BoxMeshGenerator.faces;
}