package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import org.lwjgl.opengl.GL41;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.MeshDataHolder;
import com.badlogic.gdx.graphics.*;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;

public class RDXMeshDataInterpreter
{
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