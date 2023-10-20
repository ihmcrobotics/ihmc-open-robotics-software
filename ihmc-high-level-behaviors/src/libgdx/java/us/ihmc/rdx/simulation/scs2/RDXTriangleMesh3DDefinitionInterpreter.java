package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;

import java.lang.reflect.Array;
import java.util.HashMap;
import java.util.Map;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;
import static com.badlogic.gdx.graphics.VertexAttributes.Usage.TextureCoordinates;

public class RDXTriangleMesh3DDefinitionInterpreter
{
   public static Mesh interpretDefinition(GeometryDefinition definition)
   {
      return interpretDefinition(TriangleMesh3DFactories.TriangleMesh(definition));
   }

   public static Mesh interpretDefinition(TriangleMesh3DDefinition definition)
   {
      return interpretDefinition(definition, true);
   }

   public static Mesh interpretDefinition(TriangleMesh3DDefinition definition, boolean optimizeMesh)
   {
      if (definition == null || definition.getTriangleIndices().length == 0)
         return null;

      Point3D32[] vertices = definition.getVertices();
      Point2D32[] texturePoints = definition.getTextures();
      int[] triangleIndices = definition.getTriangleIndices();
      Vector3D32[] normals = definition.getNormals();
      TIntArrayList facesIndices = new TIntArrayList();

      if (optimizeMesh)
      {
         Pair<int[], Point3D32[]> filterDuplicateVertices = filterDuplicates(triangleIndices, vertices);
         Pair<int[], Vector3D32[]> filterDuplicateNormals = filterDuplicates(triangleIndices, normals);
         Pair<int[], Point2D32[]> filterDuplicateTex = filterDuplicates(triangleIndices, texturePoints);
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

      TexCoord2f[] outputTexturePoints = new TexCoord2f[texturePoints.length];
      for (int i = 0; i < texturePoints.length; i++)
         outputTexturePoints[i] = new TexCoord2f(texturePoints[i].getX32(), texturePoints[i].getY32());
      MeshDataHolder meshData = new MeshDataHolder(vertices, outputTexturePoints, triangleIndices, normals);

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL20.GL_TRIANGLES);

      for (int i = 0; i < meshData.getVertices().length; i++)
      {
         Vector3 position = LibGDXTools.toLibGDX(meshData.getVertices()[i]);
         Vector3 normal = LibGDXTools.toLibGDX(meshData.getVertexNormals()[i]);
         Color color = Color.WHITE;
         Vector2 uvTextureCoordinates = LibGDXTools.toLibGDX(meshData.getTexturePoints()[i]);
         meshBuilder.vertex(position, normal, color, uvTextureCoordinates);
      }

      for (int i = 0; i < meshData.getTriangleIndices().length; i += 3)
      {
         meshBuilder.triangle((short) meshData.getTriangleIndices()[i],
                              (short) meshData.getTriangleIndices()[i + 1],
                              (short) meshData.getTriangleIndices()[i + 2]);
      }

      return meshBuilder.end();
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
         if (tuple == null)
         {
            LogTools.error("Got Null, Something is funny here");
            array[index++] = Float.NaN;
            array[index++] = Float.NaN;
            array[index++] = Float.NaN;
         }
         else
         {
            array[index++] = tuple.getX32();
            array[index++] = tuple.getY32();
            array[index++] = tuple.getZ32();
         }
      }
      return array;
   }

   private static float[] convertToFloatArray(Tuple2DBasics[] tuple2fs)
   {
      float[] array = new float[2 * tuple2fs.length];
      int index = 0;
      for (Tuple2DBasics tuple : tuple2fs)
      {
         if (tuple == null)
         {
            LogTools.error("Got Null, Something is funny here");
            array[index++] = Float.NaN;
            array[index++] = Float.NaN;
         }
         else
         {
            array[index++] = tuple.getX32();
            array[index++] = tuple.getY32();
         }
      }
      return array;
   }
}
