package us.ihmc.javaFXToolkit.graphics;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Tuple2f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import gnu.trove.list.array.TIntArrayList;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;

public class JavaFXMeshDataInterpreter
{
   public static TriangleMesh interpretMeshData(MeshDataHolder meshData)
   {
      if (meshData == null)
         return null;

      Point3f[] vertices = meshData.getVertices();
      TexCoord2f[] texturePoints = meshData.getTexturePoints();
      int[] triangleIndices = meshData.getTriangleIndices();
      Vector3f[] polygonNormals = meshData.getVertexNormals();

      TIntArrayList facesVertexOnlyIndices = new TIntArrayList();
      TIntArrayList facesIndices = new TIntArrayList();

      facesVertexOnlyIndices.addAll(triangleIndices);
      for (int triangleVertexIndex : triangleIndices)
      {
         facesIndices.add(triangleVertexIndex); // vertex index
         facesIndices.add(triangleVertexIndex); // normal index
         facesIndices.add(triangleVertexIndex); // texture index
      }

      int[] indices = facesIndices.toArray();

      TriangleMesh triangleMesh = new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
      triangleMesh.getPoints().addAll(convertToFloatArray(vertices));
      triangleMesh.getTexCoords().addAll(convertToFloatArray(texturePoints));
      triangleMesh.getFaces().addAll(indices);
      triangleMesh.getFaceSmoothingGroups().addAll(new int[indices.length / triangleMesh.getFaceElementSize()]);
      triangleMesh.getNormals().addAll(convertToFloatArray(polygonNormals));

      return triangleMesh;
   }

   private static float[] convertToFloatArray(Tuple3f[] tuple3fs)
   {
      float[] array = new float[3 * tuple3fs.length];
      int index = 0;
      for (Tuple3f tuple : tuple3fs)
      {
         array[index++] = tuple.getX();
         array[index++] = tuple.getY();
         array[index++] = tuple.getZ();
      }
      return array;
   }

   private static float[] convertToFloatArray(Tuple2f[] tuple2fs)
   {
      float[] array = new float[2 * tuple2fs.length];
      int index = 0;
      for (Tuple2f tuple : tuple2fs)
      {
         array[index++] = tuple.getX();
         array[index++] = tuple.getY();
      }
      return array;
   }
}
