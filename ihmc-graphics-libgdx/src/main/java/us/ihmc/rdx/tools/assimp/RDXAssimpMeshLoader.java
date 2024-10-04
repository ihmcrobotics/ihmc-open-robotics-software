package us.ihmc.rdx.tools.assimp;

import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMesh;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.utils.Array;
import gnu.trove.list.array.TShortArrayList;
import org.lwjgl.assimp.AIFace;
import org.lwjgl.assimp.AIMesh;
import org.lwjgl.assimp.AIVector3D;
import org.lwjgl.assimp.Assimp;
import us.ihmc.log.LogTools;

import java.nio.FloatBuffer;
import java.util.HashMap;

public class RDXAssimpMeshLoader
{
   private final AIMesh assimpMesh;
   private boolean hasNormals;
   private boolean hasTangents;
   private boolean hasBitangents;
   private boolean hasColors;
   private boolean hasTextureCoordinates;
   private int numberOfVertices;
   private ModelMesh modelMesh;

   private static final HashMap<String, Integer> sameMeshIds = new HashMap<>();

   public RDXAssimpMeshLoader(AIMesh assimpMesh)
   {
      this.assimpMesh = assimpMesh;
   }

   public ModelMesh load()
   {
      modelMesh = new ModelMesh();
      String meshName = assimpMesh.mName().dataString().trim();

      // Ensure meshes have unique names, which happens especially when meshes get broken up due to size by assimp
      String originalMeshName = meshName;
      int indexOfThisName = sameMeshIds.computeIfAbsent(meshName, key -> 0);
      if (indexOfThisName > 0)
         meshName = meshName + indexOfThisName;
      sameMeshIds.put(originalMeshName, indexOfThisName + 1);

      LogTools.debug("Mesh name: {}", meshName);
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

      int primitiveTypes = assimpMesh.mPrimitiveTypes();
      int gdxPrimitiveType = getGdxPrimitiveType(primitiveTypes);

      int numberOfFaces = assimpMesh.mNumFaces();
      LogTools.debug("Number of faces: {}", numberOfFaces);

      modelMeshPart.primitiveType = gdxPrimitiveType;
      TShortArrayList indexArray = new TShortArrayList();
      for (int i = 0; i < numberOfFaces; i++)
      {
         // We are assuming that faces have 3 indices each
         // TODO: Check on Assimp.AI_SCENE_FLAGS_NON_VERBOSE_FORMAT, where face indices are compacted
         AIFace assimpFace = assimpMesh.mFaces().get(i);
         for (int j = 0; j < assimpFace.mNumIndices(); j++)
         {
            int faceVertexIndex = assimpFace.mIndices().get(j);
            indexArray.add((short) faceVertexIndex);
         }
      }
      modelMeshPart.indices = indexArray.toArray();
      LogTools.debug("Number of indices: {}", modelMeshPart.indices.length);

      parts.add(modelMeshPart);
      modelMesh.parts = parts.toArray(ModelMeshPart.class);

      return modelMesh;
   }

   private int getGdxPrimitiveType(int primitiveTypes)
   {
      int gdxPrimitiveType;
      if (primitiveTypes == Assimp.aiPrimitiveType_POINT)
      {
         gdxPrimitiveType = GL20.GL_POINTS;
         String message = "Points not implemented!";
         LogTools.error(message);
         throw new RuntimeException(message);
      }
      else if (primitiveTypes == Assimp.aiPrimitiveType_LINE)
      {
         gdxPrimitiveType = GL20.GL_LINES;
         String message = "Lines not implemented!";
         LogTools.error(message);
         throw new RuntimeException(message);
      }
      else if (primitiveTypes == Assimp.aiPrimitiveType_TRIANGLE)
      {
         gdxPrimitiveType = GL20.GL_TRIANGLES;
      }
      else if (primitiveTypes == Assimp.aiPrimitiveType_POLYGON)
      {
         String message = "Polygons not implemented!";
         LogTools.error(message);
         throw new RuntimeException(message);
      }
      else
      {
         gdxPrimitiveType = Assimp.aiTextureMapping_OTHER;
         LogTools.warn("Undefined assimp primitive type");
      }
      return gdxPrimitiveType;
   }

   private VertexAttribute[] loadAttributes()
   {
      // vertices and faces guaranteed to present; else need to check null
      Array<VertexAttribute> vertexAttributes = new Array<>();
      numberOfVertices = assimpMesh.mNumVertices();
      LogTools.debug("Number of vertices: {}", numberOfVertices);
      if (numberOfVertices > MeshBuilder.MAX_VERTICES)
         LogTools.error("Mesh contains too many vertices! {}/{}", numberOfVertices, MeshBuilder.MAX_VERTICES);
      vertexAttributes.add(VertexAttribute.Position());

      hasNormals = assimpMesh.mNormals() != null;
      if (hasNormals)
      {
         vertexAttributes.add(VertexAttribute.Normal());
      }
      LogTools.debug("Has normals: {}", hasNormals);

      hasTangents = assimpMesh.mTangents() != null;
      if (hasTangents)
      {
         vertexAttributes.add(VertexAttribute.Tangent());
      }
      LogTools.debug("Has tangents: {}", hasTangents);

      hasBitangents = assimpMesh.mTangents() != null;
      if (hasBitangents)
      {
         vertexAttributes.add(VertexAttribute.Binormal());
      }
      LogTools.debug("Has bitangents: {}", hasBitangents);

      // TODO: There actually can be up to Assimp.AI_MAX_NUMBER_OF_COLOR_SETS (4) colors per vertex
      hasColors = assimpMesh.mColors(0) != null;
      if (hasColors)
      {
         vertexAttributes.add(VertexAttribute.ColorUnpacked());
      }
      LogTools.debug("Has colors: {}", hasColors);

      // TODO: There actually can be up to Assimp.AI_MAX_NUMBER_OF_TEXTURECOORDS (4) texture coodinates per vertex
      hasTextureCoordinates = assimpMesh.mTextureCoords(0) != null;
      if (hasTextureCoordinates)
      {
         int unit = 0; // texture id?
         vertexAttributes.add(VertexAttribute.TexCoords(unit));
      }
      LogTools.debug("Has texture coordinates: {}", hasTextureCoordinates);

      return vertexAttributes.toArray(VertexAttribute.class);
   }

   private float[] loadVertices()
   {
      AIVector3D.Buffer assimpVerticesVector3DS = assimpMesh.mVertices();
      int vertexSize = 3;
      if (hasNormals)
         vertexSize += 3;
      if (hasTangents)
         vertexSize += 3;
      if (hasBitangents)
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
         if (hasTangents)
         {
            float tangentX = assimpMesh.mTangents().get(j).x();
            float tangentY = assimpMesh.mTangents().get(j).y();
            float tangentZ = assimpMesh.mTangents().get(j).z();
            vertexHeapBuffer.put(tangentX);
            vertexHeapBuffer.put(tangentY);
            vertexHeapBuffer.put(tangentZ);
         }
         if (hasBitangents)
         {
            float bitangentX = assimpMesh.mBitangents().get(j).x();
            float bitangentY = assimpMesh.mBitangents().get(j).y();
            float bitangentZ = assimpMesh.mBitangents().get(j).z();
            vertexHeapBuffer.put(bitangentX);
            vertexHeapBuffer.put(bitangentY);
            vertexHeapBuffer.put(bitangentZ);
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

   public AIMesh getAssimpMesh()
   {
      return assimpMesh;
   }

   public ModelMesh getModelMesh()
   {
      return modelMesh;
   }
}
