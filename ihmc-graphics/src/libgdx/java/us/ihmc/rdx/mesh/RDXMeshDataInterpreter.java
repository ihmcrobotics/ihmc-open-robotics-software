package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.IntIntMap;
import org.lwjgl.opengl.GL41;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.MeshDataHolder;
import com.badlogic.gdx.graphics.*;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;

import java.util.ArrayList;
import java.util.List;

public class RDXMeshDataInterpreter
{
   private static final int MAX_VERTICES_PER_MESH = 65535;

   public static Mesh interpretMeshData(MeshDataHolder meshData)
   {
      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      if (meshData.getVertexNormals() != null && meshData.getTexturePoints() != null)
      {
         int vertexCount = meshData.getVertices().length;
         int normalsCount = meshData.getVertexNormals().length;
         int texCoordsCount = meshData.getTexturePoints().length;

         if (vertexCount != normalsCount || vertexCount != texCoordsCount)
            LogTools.warn("Vertex count mismatch: vertices={}, normals={}, texCoords={}", vertexCount, normalsCount, texCoordsCount);

         vertexCount = Math.min(vertexCount, normalsCount);
         vertexCount = Math.min(vertexCount, texCoordsCount);

         for (int i = 0; i < vertexCount; i++)
         {
            Vector3 position = LibGDXTools.toLibGDX(meshData.getVertices()[i]);
            Vector3 normal = LibGDXTools.toLibGDX(meshData.getVertexNormals()[i]);
            Color color = Color.WHITE;
            Vector2 uv = LibGDXTools.toLibGDX(meshData.getTexturePoints()[i]);
            meshBuilder.vertex(position, normal, color, uv);
         }

         int[] indices = meshData.getTriangleIndices();
         for (int i = 0; i + 2 < indices.length; i += 3)
         {
            int t0 = indices[i];
            int t1 = indices[i + 1];
            int t2 = indices[i + 2];

            if (t0 < 0 || t1 < 0 || t2 < 0 || t0 >= vertexCount || t1 >= vertexCount || t2 >= vertexCount)
               continue;

            meshBuilder.triangle((short) t0, (short) t1, (short) t2);
         }
      }
      else
      {
         LogTools.warn("Mesh data contains null fields: normals={}, texCoords={}", meshData.getVertexNormals(), meshData.getTexturePoints());
      }

      return meshBuilder.end();
   }

   public static List<Mesh> interpretMeshDataToMeshes(MeshDataHolder meshData)
   {
      List<Mesh> meshes = new ArrayList<>();
      if (meshData.getVertexNormals() == null || meshData.getTexturePoints() == null)
      {
         LogTools.warn("Mesh data contains null fields: normals={}, texCoords={}", meshData.getVertexNormals(), meshData.getTexturePoints());
         return meshes;
      }

      int vertexCount = meshData.getVertices().length;
      int normalsCount = meshData.getVertexNormals().length;
      int texCoordsCount = meshData.getTexturePoints().length;

      if (vertexCount != normalsCount || vertexCount != texCoordsCount)
         LogTools.warn("Vertex count mismatch: vertices={}, normals={}, texCoords={}", vertexCount, normalsCount, texCoordsCount);

      vertexCount = Math.min(vertexCount, normalsCount);
      vertexCount = Math.min(vertexCount, texCoordsCount);

      int[] indices = meshData.getTriangleIndices();
      if (indices == null || indices.length < 3)
         return meshes;

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      IntIntMap indexMap = new IntIntMap();
      int[] currentVertexCount = new int[] {0};
      int currentTriangleCount = 0;
      boolean splitOccurred = false;

      for (int i = 0; i + 2 < indices.length; i += 3)
      {
         int t0 = indices[i];
         int t1 = indices[i + 1];
         int t2 = indices[i + 2];

         if (t0 < 0 || t1 < 0 || t2 < 0 || t0 >= vertexCount || t1 >= vertexCount || t2 >= vertexCount)
            continue;

         int needed = 0;
         if (!indexMap.containsKey(t0))
            needed++;
         if (t1 != t0 && !indexMap.containsKey(t1))
            needed++;
         if (t2 != t0 && t2 != t1 && !indexMap.containsKey(t2))
            needed++;

         if (currentVertexCount[0] + needed > MAX_VERTICES_PER_MESH)
         {
            if (currentTriangleCount > 0)
            {
               meshes.add(meshBuilder.end());
               splitOccurred = true;
            }
            else
            {
               meshBuilder.end();
            }

            meshBuilder = new MeshBuilder();
            meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);
            indexMap.clear();
            currentVertexCount[0] = 0;
            currentTriangleCount = 0;

            needed = 0;
            if (!indexMap.containsKey(t0))
               needed++;
            if (t1 != t0 && !indexMap.containsKey(t1))
               needed++;
            if (t2 != t0 && t2 != t1 && !indexMap.containsKey(t2))
               needed++;
         }

         short i0 = mapVertex(t0, meshData, meshBuilder, indexMap, currentVertexCount);
         short i1 = mapVertex(t1, meshData, meshBuilder, indexMap, currentVertexCount);
         short i2 = mapVertex(t2, meshData, meshBuilder, indexMap, currentVertexCount);
         meshBuilder.triangle(i0, i1, i2);
         currentTriangleCount++;
      }

      if (currentTriangleCount > 0)
         meshes.add(meshBuilder.end());
      else
         meshBuilder.end();

      if (splitOccurred)
         LogTools.warn("Mesh exceeded {} vertices; split into {} mesh parts.", MAX_VERTICES_PER_MESH, meshes.size());

      return meshes;
   }

   private static short mapVertex(int originalIndex,
                                  MeshDataHolder meshData,
                                  MeshBuilder meshBuilder,
                                  IntIntMap indexMap,
                                  int[] currentVertexCount)
   {
      int mapped = indexMap.get(originalIndex, -1);
      if (mapped != -1)
         return (short) mapped;

      Vector3 position = LibGDXTools.toLibGDX(meshData.getVertices()[originalIndex]);
      Vector3 normal = LibGDXTools.toLibGDX(meshData.getVertexNormals()[originalIndex]);
      Vector2 uv = LibGDXTools.toLibGDX(meshData.getTexturePoints()[originalIndex]);
      meshBuilder.vertex(position, normal, Color.WHITE, uv);

      mapped = currentVertexCount[0]++;
      indexMap.put(originalIndex, mapped);
      return (short) mapped;
   }

   public static void reorderMeshVertices(MeshDataHolder meshData, Mesh meshToPack)
   {
      meshToPack.getIndicesBuffer().clear();

      for (int i = 0; i < meshData.getTriangleIndices().length; i += 3)
      {
         meshToPack.getIndicesBuffer().put((short) meshData.getTriangleIndices()[i]);
         meshToPack.getIndicesBuffer().put((short) meshData.getTriangleIndices()[i + 1]);
         meshToPack.getIndicesBuffer().put((short) meshData.getTriangleIndices()[i + 2]);
      }

      meshToPack.getIndicesBuffer().flip();

      if (meshToPack.getVerticesBuffer().limit() == 0)
      {
         throw new RuntimeException("Mesh must have data. The application will SIGSEV on rendering if this continued.");
      }
   }

   public static void repositionMeshVertices(MeshDataHolder meshData, Mesh meshToPack, Color color)
   {
      meshToPack.getVerticesBuffer().clear();

      // Cache texture location outside the loop - it's the same for all vertices
      float[] textureLocation = RDXMultiColorMeshBuilder.getTextureLocation(color);
      float texU = textureLocation[0];
      float texV = textureLocation[1];

      for (int i = 0; i < meshData.getVertices().length; i++)
      {
         // Position
         meshToPack.getVerticesBuffer().put(meshData.getVertices()[i].getX32());
         meshToPack.getVerticesBuffer().put(meshData.getVertices()[i].getY32());
         meshToPack.getVerticesBuffer().put(meshData.getVertices()[i].getZ32());

         // ColorUnpacked
         meshToPack.getVerticesBuffer().put(Color.WHITE.r);
         meshToPack.getVerticesBuffer().put(Color.WHITE.g);
         meshToPack.getVerticesBuffer().put(Color.WHITE.b);
         meshToPack.getVerticesBuffer().put(Color.WHITE.a);

         // Normal
         meshToPack.getVerticesBuffer().put(meshData.getVertexNormals()[i].getX32());
         meshToPack.getVerticesBuffer().put(meshData.getVertexNormals()[i].getY32());
         meshToPack.getVerticesBuffer().put(meshData.getVertexNormals()[i].getZ32());

         // UV TextureCoordinates
         meshToPack.getVerticesBuffer().put(texU);
         meshToPack.getVerticesBuffer().put(texV);
      }

      meshToPack.getVerticesBuffer().flip();

      if (meshToPack.getVerticesBuffer().limit() == 0)
      {
         throw new RuntimeException("Mesh must have data. The application will SIGSEV on rendering if this continued.");
      }
   }
}
