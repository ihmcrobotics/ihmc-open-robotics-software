package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import org.lwjgl.opengl.GL41;
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

      int vertexCount       = meshData.getVertices().length;
      int normalsCount      = meshData.getVertexNormals() != null ? meshData.getVertexNormals().length : 0;
      int texCoordsCount    = meshData.getTexturePoints() != null ? meshData.getTexturePoints().length : 0;
      int safeVertexCount   = vertexCount;
      // Ensure we don't index past normals / UVs
      safeVertexCount = Math.min(safeVertexCount, normalsCount);
      safeVertexCount = Math.min(safeVertexCount, texCoordsCount);

      for (int i = 0; i < safeVertexCount; i++)
      {
         Vector3 position = LibGDXTools.toLibGDX(meshData.getVertices()[i]);

         Vector3 normal = (i < normalsCount)
               ? LibGDXTools.toLibGDX(meshData.getVertexNormals()[i])
               : new Vector3(0f, 0f, 1f); // fallback

         Vector2 uv = (i < texCoordsCount)
               ? LibGDXTools.toLibGDX(meshData.getTexturePoints()[i])
               : new Vector2(0f, 0f); // fallback

         Color color = Color.WHITE;
         meshBuilder.vertex(position, normal, color, uv);
      }

      int[] indices = meshData.getTriangleIndices();
      int maxVertexIndex = safeVertexCount; // indices must refer only to vertices we actually emitted

      for (int i = 0; i + 2 < indices.length; i += 3)
      {
         int i0 = indices[i];
         int i1 = indices[i + 1];
         int i2 = indices[i + 2];

         if (i0 < 0 || i1 < 0 || i2 < 0 ||
             i0 >= maxVertexIndex || i1 >= maxVertexIndex || i2 >= maxVertexIndex)
            continue;

         meshBuilder.triangle((short) i0, (short) i1, (short) i2);
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
         float[] textureLocation = RDXMultiColorMeshBuilder.getTextureLocation(color);
         meshToPack.getVerticesBuffer().put(textureLocation[0]);
         meshToPack.getVerticesBuffer().put(textureLocation[1]);
      }

      meshToPack.getVerticesBuffer().flip();

      if (meshToPack.getVerticesBuffer().limit() == 0)
      {
         throw new RuntimeException("Mesh must have data. The application will SIGSEV on rendering if this continued.");
      }
   }
}