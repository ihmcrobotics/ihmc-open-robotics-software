package us.ihmc.javaFXToolkit.graphics;

import java.lang.reflect.Array;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.Pair;

import gnu.trove.list.array.TIntArrayList;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;

/**
 * This class an automated interpretation of {@link MeshDataHolder} into JavaFX {@link TriangleMesh} usable via {@link MeshView}.
 * With this tool it is possible to tools from the Graphics3DDescription library such {@link MeshDataGenerator} and {@link MeshDataBuilder}.
 * It also provides a simple way to optimize meshes for JavaFX.
 * @author Sylvain Bertrand
 */
public class JavaFXMeshDataInterpreter
{
   /**
    * Translates a {@link MeshDataHolder} into {@link TriangleMesh}.
    * @param meshData the mesh data to interpret. Not modified.
    * @return the interpreted JavaFX {@link TriangleMesh}. Return {@code null} when the given mesh data is {@code null} or empty.
    */
   public static TriangleMesh interpretMeshData(MeshDataHolder meshData)
   {
      return interpretMeshData(meshData, true);
   }

   /**
    * Translates a {@link MeshDataHolder} into {@link TriangleMesh}.
    * @param meshData the mesh data to interpret. Not modified.
    * @param optimizeMesh whether to optimize the mesh data or not.
    * The optimization consists in removing duplicate vertices, texture coordinates, and vertex normals and recomputing triangle indices accordingly.
    * This process can be computationally intensive but it is often highly beneficial especially for large meshes and does not need to be executed on the rendering thread.
    * @return the interpreted JavaFX {@link TriangleMesh}. Return {@code null} when the given mesh data is {@code null} or empty.
    */
   public static TriangleMesh interpretMeshData(MeshDataHolder meshData, boolean optimizeMesh)
   {
      if (meshData == null || meshData.getTriangleIndices().length == 0)
         return null;

      Point3D32[] vertices = meshData.getVertices();
      TexCoord2f[] texturePoints = meshData.getTexturePoints();
      int[] triangleIndices = meshData.getTriangleIndices();
      Vector3D32[] normals = meshData.getVertexNormals();
      TIntArrayList facesIndices = new TIntArrayList();

      if (optimizeMesh)
      {
         Pair<int[], Point3D32[]> filterDuplicateVertices = filterDuplicates(triangleIndices, vertices);
         Pair<int[], Vector3D32[]> filterDuplicateNormals = filterDuplicates(triangleIndices, normals);
         Pair<int[], TexCoord2f[]> filterDuplicateTex = filterDuplicates(triangleIndices, texturePoints);
         vertices = filterDuplicateVertices.getRight();
         normals = filterDuplicateNormals.getRight();
         texturePoints = filterDuplicateTex.getRight();

         for (int pos = 0; pos < triangleIndices.length; pos++)
         {
            facesIndices.add(filterDuplicateVertices.getLeft()[pos]); // vertex index
            facesIndices.add(filterDuplicateNormals.getLeft()[pos]); // normal index
            facesIndices.add(filterDuplicateTex.getLeft()[pos]); // texture index
         }
      }
      else
      {
         for (int pos = 0; pos < triangleIndices.length; pos++)
         {
            facesIndices.add(triangleIndices[pos]); // vertex index
            facesIndices.add(triangleIndices[pos]); // normal index
            facesIndices.add(triangleIndices[pos]); // texture index
         }
      }

      int[] indices = facesIndices.toArray();

      TriangleMesh triangleMesh = new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
      triangleMesh.getPoints().addAll(convertToFloatArray(vertices));
      triangleMesh.getTexCoords().addAll(convertToFloatArray(texturePoints));
      triangleMesh.getFaces().addAll(indices);
      triangleMesh.getFaceSmoothingGroups().addAll(new int[indices.length / triangleMesh.getFaceElementSize()]);
      triangleMesh.getNormals().addAll(convertToFloatArray(normals));

      return triangleMesh;
   }

   private static <T> Pair<int[], T[]> filterDuplicates(int[] originalIndices, T[] valuesWithDuplicates)
   {
      Map<T, Integer> uniqueValueIndices = new HashMap<>();

      for (int valueIndex = valuesWithDuplicates.length - 1; valueIndex >= 0; valueIndex--)
         uniqueValueIndices.put(valuesWithDuplicates[valueIndex], valueIndex);

      @SuppressWarnings("unchecked")
      T[] filteredValue = (T[]) Array.newInstance(valuesWithDuplicates[0].getClass(), uniqueValueIndices.size());
      int pos = 0;

      for (T value : uniqueValueIndices.keySet())
      {
         uniqueValueIndices.put(value, pos);
         filteredValue[pos] = value;
         pos++;
      }

      int[] filteredIndices = new int[originalIndices.length];
      pos = 0;

      for (int triangleIndex : originalIndices)
         filteredIndices[pos++] = uniqueValueIndices.get(valuesWithDuplicates[triangleIndex]);

      return Pair.of(filteredIndices, filteredValue);
   }

   private static float[] convertToFloatArray(Tuple3DBasics[] tuple3fs)
   {
      float[] array = new float[3 * tuple3fs.length];
      int index = 0;
      for (Tuple3DBasics tuple : tuple3fs)
      {
         array[index++] = tuple.getX32();
         array[index++] = tuple.getY32();
         array[index++] = tuple.getZ32();
      }
      return array;
   }

   private static float[] convertToFloatArray(Tuple2DBasics[] tuple2fs)
   {
      float[] array = new float[2 * tuple2fs.length];
      int index = 0;
      for (Tuple2DBasics tuple : tuple2fs)
      {
         array[index++] = tuple.getX32();
         array[index++] = tuple.getY32();
      }
      return array;
   }
}
