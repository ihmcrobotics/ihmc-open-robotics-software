package us.ihmc.gdx.tools.assimp;

import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMesh;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMeshPart;
import com.badlogic.gdx.utils.Array;
import org.lwjgl.assimp.AIFace;
import org.lwjgl.assimp.AIMesh;
import org.lwjgl.assimp.AIVector3D;
import us.ihmc.log.LogTools;

import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

public class GDXAssimpMeshLoader
{
   private final AIMesh assimpMesh;
   private boolean hasNormals;
   private boolean hasColors;
   private boolean hasTextureCoordinates;
   private int numberOfVertices;

   public GDXAssimpMeshLoader(AIMesh assimpMesh)
   {
      this.assimpMesh = assimpMesh;
   }

   public ModelMesh load(int index)
   {
      ModelMesh modelMesh = new ModelMesh();
      String meshName = assimpMesh.mName().dataString().trim();
      LogTools.info("Mesh name: {}", meshName);
      if (!meshName.isEmpty())
         modelMesh.id = meshName;
//      else
//         modelMesh.id = "mesh" + index;
      else
         modelMesh.id = "";

      modelMesh.attributes = loadAttributes();
      modelMesh.vertices = loadVertices();

      // Just one part for now
      Array<ModelMeshPart> parts = new Array<>();
      ModelMeshPart modelMeshPart = new ModelMeshPart();
      modelMeshPart.id = meshName;

      int numberOfFaces = assimpMesh.mNumFaces();
      LogTools.info("Number of faces: {}", numberOfFaces);

      if (numberOfFaces > 0)
      {
         int numberOfIndices = assimpMesh.mFaces().get(0).mNumIndices();
         LogTools.info("Number of indices: {}", numberOfIndices);
         if (numberOfIndices != 3)
            LogTools.error("Number of indices not implemented");
      }

      modelMeshPart.primitiveType = GL20.GL_TRIANGLES;
      ShortBuffer indexBuffer = ShortBuffer.allocate(numberOfFaces * 3 * Short.BYTES);
      for (int i = 0; i < numberOfFaces; i++)
      {
         // We are assuming that faces have 3 indices each
         AIFace assimpFace = assimpMesh.mFaces().get(i);
         int index0 = assimpFace.mIndices().get(0);
         int index1 = assimpFace.mIndices().get(1);
         int index2 = assimpFace.mIndices().get(2);
         indexBuffer.put((short) index0);
         indexBuffer.put((short) index1);
         indexBuffer.put((short) index2);
      }
      modelMeshPart.indices = indexBuffer.array();

      parts.add(modelMeshPart);
      modelMesh.parts = parts.toArray(ModelMeshPart.class);

      return modelMesh;
   }

   private VertexAttribute[] loadAttributes()
   {
      // vertices and faces guaranteed to present; else need to check null
      Array<VertexAttribute> vertexAttributes = new Array<>();
      numberOfVertices = assimpMesh.mNumVertices();
      LogTools.info("Number of vertices: {}", numberOfVertices);
      vertexAttributes.add(VertexAttribute.Position());

      hasNormals = assimpMesh.mNormals() != null;
      if (hasNormals)
      {
         vertexAttributes.add(VertexAttribute.Normal());
      }
      LogTools.info("Has normals: {}", hasNormals);

      // TODO: There actually can be up to Assimp.AI_MAX_NUMBER_OF_COLOR_SETS (4) colors per vertex
      hasColors = assimpMesh.mColors(0) != null;
      if (hasColors)
      {
         vertexAttributes.add(VertexAttribute.ColorUnpacked());
      }
      LogTools.info("Has colors: {}", hasColors);

      // TODO: There actually can be up to Assimp.AI_MAX_NUMBER_OF_TEXTURECOORDS (4) texture coodinates per vertex
      hasTextureCoordinates = assimpMesh.mTextureCoords(0) != null;
      if (hasTextureCoordinates)
      {
         int unit = 0; // texture id?
         vertexAttributes.add(VertexAttribute.TexCoords(unit));
      }
      LogTools.info("Has texture coordinates: {}", hasTextureCoordinates);

      return vertexAttributes.toArray(VertexAttribute.class);
   }

   private float[] loadVertices()
   {
      AIVector3D.Buffer assimpVerticesVector3DS = assimpMesh.mVertices();
      int vertexSize = 3;
      if (hasNormals)
         vertexSize += 3;
      if (hasColors)
         vertexSize += 4;
      if (hasTextureCoordinates)
         vertexSize += 2;
      FloatBuffer vertexHeapBuffer = FloatBuffer.allocate(numberOfVertices * vertexSize);
      for (int j = 0; j < numberOfVertices; j++)
      {
         float x = assimpVerticesVector3DS.get(j).x();
         float y = assimpVerticesVector3DS.get(j).y();
         float z = assimpVerticesVector3DS.get(j).z();
         vertexHeapBuffer.put(x);
         vertexHeapBuffer.put(y);
         vertexHeapBuffer.put(z);

         if (hasNormals)
         {
            float normalX = assimpMesh.mNormals().get(j).x();
            float normalY = assimpMesh.mNormals().get(j).y();
            float normalZ = assimpMesh.mNormals().get(j).z();
            vertexHeapBuffer.put(normalX);
            vertexHeapBuffer.put(normalY);
            vertexHeapBuffer.put(normalZ);
         }
         if (hasColors)
         {
            float r = assimpMesh.mColors(0).get(j).r();
            float g = assimpMesh.mColors(0).get(j).g();
            float b = assimpMesh.mColors(0).get(j).b();
            float a = assimpMesh.mColors(0).get(j).a();
            vertexHeapBuffer.put(r);
            vertexHeapBuffer.put(g);
            vertexHeapBuffer.put(b);
            vertexHeapBuffer.put(a);
         }
         if (hasTextureCoordinates)
         {
            // Assume 2D texture here
            float textureX = assimpMesh.mTextureCoords(0).get(j).x();
            float textureY = assimpMesh.mTextureCoords(0).get(j).y();
            vertexHeapBuffer.put(textureX);
            vertexHeapBuffer.put(textureY);
         }
      }
      return vertexHeapBuffer.array();
   }
}
