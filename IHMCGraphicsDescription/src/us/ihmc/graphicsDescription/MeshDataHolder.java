package us.ihmc.graphicsDescription;

import java.lang.reflect.Array;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;

/**
 * This class provides an immutable data structure for 3D graphic mesh that is independent from the graphics engine to be used.
 * It contains all the data necessary to create a mesh.
 * Using the corresponding mesh data interpreter, a {@link MeshDataHolder} can be translated into a specific mesh data type usable by a specific graphics engine such as JME or JavaFX.
 */
public class MeshDataHolder
{
   private final Point3D32[] vertices;
   private final TexCoord2f[] texturePoints;
   private final int[] triangleIndices;
   private final Vector3D32[] vertexNormals;
   private String name = "MeshDataHolder";

   /**
    * Default construct to create a mesh data.
    * @param vertices the 3D coordinates of the mesh vertices.
    * @param texturePoints the 2D texture coordinates to be used for each vertex of the mesh.
    * @param triangleIndices a list of triplet indices. Each triplet describes the three indices used to pick the three vertex coordinates, texture coordinates, and normal coordinates to render a 3D triangle.
    * @param vertexNormals the 3D normal coordinates to be used for each vertex of the mesh.
    */
   public MeshDataHolder(Point3D32[] vertices, TexCoord2f[] texturePoints, int[] triangleIndices, Vector3D32[] vertexNormals)
   {
      this.vertices = vertices;
      this.texturePoints = texturePoints;
      this.triangleIndices = triangleIndices;
      this.vertexNormals = vertexNormals;
   }

   /**
    * @return the 3D coordinates of the mesh vertices.
    */
   public Point3D32[] getVertices()
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
   public Vector3D32[] getVertexNormals()
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
   public static MeshDataHolder rotate(MeshDataHolder input, RotationMatrix matrix)
   {
      TexCoord2f[] texturePoints = input.getTexturePoints();
      int[] triangleIndices = input.getTriangleIndices();
      Point3D32[] inputVertices = input.getVertices();
      Vector3D32[] inputNormals = input.getVertexNormals();

      Point3D32[] outputVertices = new Point3D32[inputVertices.length];
      Vector3D32[] outputNormals = new Vector3D32[inputNormals.length];

      for (int i = 0; i < inputVertices.length; i++)
      {
         outputVertices[i] = new Point3D32();
         outputNormals[i] = new Vector3D32();
         matrix.transform(inputVertices[i], outputVertices[i]);
         matrix.transform(inputNormals[i], outputNormals[i]);
      }
      return new MeshDataHolder(outputVertices, texturePoints, triangleIndices, outputNormals);
   }

   /**
    * Utility method to rotate a given mesh using a given rotation matrix.
    * @param input the mesh to rotate. Not modified.
    * @param axisAngle the axis-angle describing the rotation to apply to the mesh. Not Modified.
    * @return the rotated mesh.
    */
   public static MeshDataHolder rotate(MeshDataHolder input, AxisAngle axisAngle)
   {
      RotationMatrix matrix = new RotationMatrix();
      matrix.set(axisAngle);
      return rotate(input, matrix);
   }

   /**
    * Utility method to rotate a given mesh using a given rotation matrix.
    * @param input the mesh to rotate. Not modified.
    * @param quaternion the quaternion describing the rotation to apply to the mesh. Not Modified.
    * @return the rotated mesh.
    */
   public static MeshDataHolder rotate(MeshDataHolder input, Quaternion32 quaternion)
   {
      RotationMatrix matrix = new RotationMatrix();
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
      Point3D32[] inputVertices = input.getVertices();
      TexCoord2f[] texturePoints = input.getTexturePoints();
      int[] triangleIndices = input.getTriangleIndices();
      Vector3D32[] normals = input.getVertexNormals();

      Point3D32[] outputVertices = new Point3D32[inputVertices.length];
      for (int i = 0; i < inputVertices.length; i++)
      {
         outputVertices[i] = new Point3D32(offsetX, offsetY, offsetZ);
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
   public static MeshDataHolder translate(MeshDataHolder input, Tuple3DReadOnly offset)
   {
      return translate(input, offset.getX32(), offset.getY32(), offset.getZ32());
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
      Point3D32[] vertices = combineArrays(meshData1.vertices, meshData2.vertices);
      TexCoord2f[] texturePoints = combineArrays(meshData1.texturePoints, meshData2.texturePoints);
      Vector3D32[] vertexNormals = combineArrays(meshData1.vertexNormals, meshData2.vertexNormals);
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
