package us.ihmc.javaFXToolkit.shapes.meshGenerators;

import java.util.Arrays;

public class BoxMeshGenerator
{
   public static FXMeshDataHolder cubeMesh(double size)
   {
      return boxMesh(size, size, size);
   }
   public static FXMeshDataHolder cubeMesh(float size)
   {
      return boxMesh(size, size, size);
   }

   public static FXMeshDataHolder boxMesh(double lx, double ly, double lz)
   {
      return boxMesh((float) lx, (float) ly, (float) lz);
   }

   public static FXMeshDataHolder boxMesh(float lx, float ly, float lz)
   {
      FXMeshDataHolder boxMesh = new FXMeshDataHolder();
      boxMesh.setVertexCoordinates(generatePoints(lx, ly, lz));
      boxMesh.setTextureCoordinates(Arrays.copyOf(texCoords, texCoords.length));
      boxMesh.setFaceIndices(Arrays.copyOf(faces, faces.length));
      boxMesh.setFaceSmoothingGroups(Arrays.copyOf(faceSmoothingGroups, faceSmoothingGroups.length));
      return boxMesh;
   }

   public static float[] generatePoints(float lx, float ly, float lz)
   {
      float halfLx = lx / 2.0f;
      float halfLy = ly / 2.0f;
      float halfLz = lz / 2.0f;

      float points[] = {-halfLx, -halfLy, -halfLz, halfLx, -halfLy, -halfLz, halfLx, halfLy, -halfLz, -halfLx, halfLy, -halfLz, -halfLx, -halfLy, halfLz,
            halfLx, -halfLy, halfLz, halfLx, halfLy, halfLz, -halfLx, halfLy, halfLz};
      return points;
   }

   public static final float texCoords[] = {0, 0, 1, 0, 1, 1, 0, 1};

   public static final int faceSmoothingGroups[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

   public static final int faces[] = {0, 0, 2, 2, 1, 1, 2, 2, 0, 0, 3, 3, 1, 0, 6, 2, 5, 1, 6, 2, 1, 0, 2, 3, 5, 0, 7, 2, 4, 1, 7, 2, 5, 0, 6, 3, 4, 0, 3, 2, 0, 1,
         3, 2, 4, 0, 7, 3, 3, 0, 6, 2, 2, 1, 6, 2, 3, 0, 7, 3, 4, 0, 1, 2, 5, 1, 1, 2, 4, 0, 0, 3,};
}