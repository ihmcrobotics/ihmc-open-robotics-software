package us.ihmc.graphicsDescription;

import java.lang.reflect.Array;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

/**
 * This class provides an immutable data structure for 3D graphic mesh that is independent from the graphics engine to be used.
 * It contains all the data necessary to create a mesh.
 * Using the corresponding mesh data interpreter, a {@link MeshDataHolder} can be translated into a specific mesh data type usable by a specific graphics engine such as JME or JavaFX.
 */
public class MeshDataHolder
{
   private final Point3f[] vertices;
   private final TexCoord2f[] texturePoints;
   private final int[] triangleIndices;
   private final Vector3f[] vertexNormals;
   private String name = "MeshDataHolder";

   /**
    * Default construct to create a mesh data.
    * @param vertices the 3D coordinates of the mesh vertices.
    * @param texturePoints the 2D texture coordinates to be used for each vertex of the mesh.
    * @param triangleIndices a list of triplet indices. Each triplet describes the three indices used to pick the three vertex coordinates, texture coordinates, and normal coordinates to render a 3D triangle.
    * @param vertexNormals the 3D normal coordinates to be used for each vertex of the mesh.
    */
   public MeshDataHolder(Point3f[] vertices, TexCoord2f[] texturePoints, int[] triangleIndices, Vector3f[] vertexNormals)
   {
      this.vertices = vertices;
      this.texturePoints = texturePoints;
      this.triangleIndices = triangleIndices;
      this.vertexNormals = vertexNormals;
   }

   /**
    * @return the 3D coordinates of the mesh vertices.
    */
   public Point3f[] getVertices()
   {
      return vertices;
   }

   /**
    * @return the 2D texture coordinates to be used for each vertex of the mesh.
    */
   public TexCoord2f[] getTexturePoints()
   {
      return texturePoints;
   }

   /**
    * @return a list of triplet indices. Each triplet describes the three indices used to pick the three vertex coordinates, texture coordinates, and normal coordinates to render a 3D triangle.
    */
   public int[] getTriangleIndices()
   {
      return triangleIndices;
   }

   /**
    * @return the 3D normal coordinates to be used for each vertex of the mesh.
    */
   public Vector3f[] getVertexNormals()
   {
      return vertexNormals;
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   /**
    * Utility method to rotate a given mesh using a given rotation matrix.
    * @param input the mesh to rotate. Not modified.
    * @param matrix the rotation to apply to the mesh. Not Modified.
    * @return the rotated mesh.
    */
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

   /**
    * Utility method to rotate a given mesh using a given rotation matrix.
    * @param input the mesh to rotate. Not modified.
    * @param matrix the rotation to apply to the mesh. Not Modified.
    * @return the rotated mesh.
    */
   public static MeshDataHolder rotate(MeshDataHolder input, Matrix3d matrix)
   {
      return rotate(input, new Matrix3f(matrix));
   }

   /**
    * Utility method to rotate a given mesh using a given rotation matrix.
    * @param input the mesh to rotate. Not modified.
    * @param axisAngle the axis-angle describing the rotation to apply to the mesh. Not Modified.
    * @return the rotated mesh.
    */
   public static MeshDataHolder rotate(MeshDataHolder input, AxisAngle4d axisAngle)
   {
      Matrix3f matrix = new Matrix3f();
      matrix.set(axisAngle);
      return rotate(input, matrix);
   }

   /**
    * Utility method to rotate a given mesh using a given rotation matrix.
    * @param input the mesh to rotate. Not modified.
    * @param quaternion the quaternion describing the rotation to apply to the mesh. Not Modified.
    * @return the rotated mesh.
    */
   public static MeshDataHolder rotate(MeshDataHolder input, Quat4f quaternion)
   {
      Matrix3f matrix = new Matrix3f();
      matrix.set(quaternion);
      return rotate(input, matrix);
   }

   /**
    * Utility method to translate a given mesh using a given translation.
    * @param input the mesh to translate. Not modified.
    * @param offsetX translation along the x-axis to apply to the mesh.
    * @param offsetY translation along the y-axis to apply to the mesh.
    * @param offsetZ translation along the z-axis to apply to the mesh.
    * @return the translated mesh.
    */
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

   /**
    * Utility method to translate a given mesh using a given translation.
    * @param input the mesh to translate. Not modified.
    * @param offset translation to apply to the mesh. Not modified.
    * @return the translated mesh.
    */
   public static MeshDataHolder translate(MeshDataHolder input, Tuple3f offset)
   {
      return translate(input, offset.getX(), offset.getY(), offset.getZ());
   }

   public static MeshDataHolder translate(MeshDataHolder input, Tuple3d offset)
   {
      return translate(input, (float) offset.getX(), (float) offset.getY(), (float) offset.getZ());
   }

   /**
    * Utility method to combine two meshes into one by concatenation: (pseudo-code) {@code result = [mesh1, mesh2]}.
    * @param meshData1 the first mesh to combine. Not modified.
    * @param meshData2 the second mesh to combine. Not modified.
    * @param updateMeshData2TrianglesIndices whether the triangle indices of the second mesh should be updated during the operation.
    *        Highly recommended, set it to false only if you what you are doing.
    * @return a new mesh resulting from the combination.
    */
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
