package us.ihmc.graphics3DAdapter.graphics;

import java.lang.reflect.Array;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
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

   public static MeshDataHolder rotate(MeshDataHolder input, Matrix3f matrix)
   {
      TexCoord2f[] texturePoints = input.getTexturePoints();
      int[] triangleIndices = input.getTriangleIndices();
      Point3f[] inputVertices = input.getVertices();
      Vector3f[] inputNormals = input.getVertexNormals();
   
      Point3f[] outputVertices = new Point3f[inputVertices.length];
      Vector3f[] outputNormals = new Vector3f[inputNormals.length];
   
      for (int i = 0; i < inputVertices.length; i++)
      {
         outputVertices[i] = new Point3f();
         outputNormals[i] = new Vector3f();
         matrix.transform(inputVertices[i], outputVertices[i]);
         matrix.transform(inputNormals[i], outputNormals[i]);
      }
      return new MeshDataHolder(outputVertices, texturePoints, triangleIndices, outputNormals);
   }

   public static MeshDataHolder rotate(MeshDataHolder input, Matrix3d matrix)
   {
      return rotate(input, new Matrix3f(matrix));
   }

   public static MeshDataHolder rotate(MeshDataHolder input, AxisAngle4d axisAngle)
   {
      Matrix3f matrix = new Matrix3f();
      matrix.set(axisAngle);
      return rotate(input, matrix);
   }

   public static MeshDataHolder translate(MeshDataHolder input, float offsetX, float offsetY, float offsetZ)
   {
      Point3f[] inputVertices = input.getVertices();
      TexCoord2f[] texturePoints = input.getTexturePoints();
      int[] triangleIndices = input.getTriangleIndices();
      Vector3f[] normals = input.getVertexNormals();
   
      Point3f[] outputVertices = new Point3f[inputVertices.length];
      for (int i = 0; i < inputVertices.length; i++)
      {
         outputVertices[i] = new Point3f(offsetX, offsetY, offsetZ);
         outputVertices[i].add(inputVertices[i]);
      }
      return new MeshDataHolder(outputVertices, texturePoints, triangleIndices, normals);
   }

   public static MeshDataHolder translate(MeshDataHolder input, Tuple3f offset)
   {
      return translate(input, offset.getX(), offset.getY(), offset.getZ());
   }

   public static MeshDataHolder translate(MeshDataHolder input, Tuple3d offset)
   {
      return translate(input, (float) offset.getX(), (float) offset.getY(), (float) offset.getZ());
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
