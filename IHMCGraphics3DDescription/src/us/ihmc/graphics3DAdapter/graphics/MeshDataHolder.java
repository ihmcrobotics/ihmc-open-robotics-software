package us.ihmc.graphics3DAdapter.graphics;

import java.lang.reflect.Array;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3f;

public class MeshDataHolder
{
   private final Point3f[] vertices;
   private final TexCoord2f[] texturePoints;
   private final int[] triangleIndices;
   private final Vector3f[] vertexNormals;

   public MeshDataHolder(Point3f[] vertices, TexCoord2f[] texturePoints, int[] triangleIndices, Vector3f[] vertexNormals)
   {
      this.vertices = vertices;
      this.texturePoints = texturePoints;
      this.triangleIndices = triangleIndices;
      this.vertexNormals = vertexNormals;
   }

   public Point3f[] getVertices()
   {
      return vertices;
   }

   public TexCoord2f[] getTexturePoints()
   {
      return texturePoints;
   }

   public int[] getTriangleIndices()
   {
      return triangleIndices;
   }

   public Vector3f[] getVertexNormals()
   {
      return vertexNormals;
   }

   public static MeshDataHolder combine(MeshDataHolder meshData1, MeshDataHolder meshData2, boolean updateMeshData2TrianglesIndices)
   {
      Point3f[] vertices = combineArrays(meshData1.vertices, meshData2.vertices);
      TexCoord2f[] texturePoints = combineArrays(meshData1.texturePoints, meshData2.texturePoints);
      Vector3f[] vertexNormals = combineArrays(meshData1.vertexNormals, meshData2.vertexNormals);
      int[] triangleIndices = combineArrays(meshData1.triangleIndices, meshData2.triangleIndices);

      if (updateMeshData2TrianglesIndices)
      {
         int shift = meshData1.vertices.length;
         for (int i = meshData1.triangleIndices.length; i < triangleIndices.length; i++)
            triangleIndices[i] += shift;
      }

      return new MeshDataHolder(vertices, texturePoints, triangleIndices, vertexNormals);
   }

   private static <T> T[] combineArrays(T[] array1, T[] array2)
   {
      @SuppressWarnings("unchecked")
      T[] combined = (T[]) Array.newInstance(array1[0].getClass(), array1.length + array2.length);
      System.arraycopy(array1, 0, combined, 0, array1.length);
      System.arraycopy(array2, 0, combined, array1.length, array2.length);
      return combined;
   }

   private static int[] combineArrays(int[] array1, int[] array2)
   {
      int[] combined = new int[array1.length + array2.length];
      System.arraycopy(array1, 0, combined, 0, array1.length);
      System.arraycopy(array2, 0, combined, array1.length, array2.length);
      return combined;
   }
}
