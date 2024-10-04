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