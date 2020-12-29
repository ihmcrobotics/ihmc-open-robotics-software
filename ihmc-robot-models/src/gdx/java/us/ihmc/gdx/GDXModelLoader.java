package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.FloatAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import com.badlogic.gdx.graphics.g3d.model.data.*;
import com.badlogic.gdx.graphics.g3d.utils.TextureDescriptor;
import com.badlogic.gdx.graphics.g3d.utils.TextureProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.*;

public class GDXModelLoader
{
   public static Model loadG3DModel(String modelFileName)
   {
      //         G3dModelLoader g3dModelLoader = new G3dModelLoader(new JsonReader(), new InternalFileHandleResolver());
      //         FileHandle modelFile = Gdx.files.internal(modelFileName);
      //         Model model = g3dModelLoader.loadModel(modelFile, new TextureProvider.FileTextureProvider());

      return new G3dModelLoader(new JsonReader()).loadModel(Gdx.files.internal(modelFileName));
   }

//   public static void loadG3DModelOtherWay(String modelFileName, Node node)
//   {
//
//      G3dModelLoader g3dModelLoader = new G3dModelLoader(new JsonReader());
//      FileHandle fileHandle = Gdx.files.internal(modelFileName);
//      TextureProvider.FileTextureProvider textureProvider = new TextureProvider.FileTextureProvider();
//
//      ModelData modelData = g3dModelLoader.parseModel(fileHandle);
//
//      for (ModelMesh mesh : modelData.meshes)
//      {
//         for (ModelMeshPart part : mesh.parts)
//         {
//
//            new MeshPart();
//            node.parts.add(new NodePart());
//         }
//      }
//   }
//
//   private static void loadMeshes(Iterable<ModelMesh> meshes, Node node)
//   {
//      for (ModelMesh mesh : meshes)
//      {
//         convertMesh(mesh, node);
//      }
//   }
//
//   private static void convertMesh(ModelMesh modelMesh, Node node)
//   {
//      int numIndices = 0;
//      for (ModelMeshPart part : modelMesh.parts)
//      {
//         numIndices += part.indices.length;
//      }
//      boolean hasIndices = numIndices > 0;
//      VertexAttributes attributes = new VertexAttributes(modelMesh.attributes);
//      int numVertices = modelMesh.vertices.length / (attributes.vertexSize / 4);
//
//      Mesh mesh = new Mesh(true, numVertices, numIndices, attributes);
//      meshes.add(mesh);
//      disposables.add(mesh);
//
//      BufferUtils.copy(modelMesh.vertices, mesh.getVerticesBuffer(), modelMesh.vertices.length, 0);
//      int offset = 0;
//      mesh.getIndicesBuffer().clear();
//      for (ModelMeshPart part : modelMesh.parts)
//      {
//         MeshPart meshPart = new MeshPart();
//         meshPart.id = part.id;
//         meshPart.primitiveType = part.primitiveType;
//         meshPart.offset = offset;
//         meshPart.size = hasIndices ? part.indices.length : numVertices;
//         meshPart.mesh = mesh;
//         if (hasIndices)
//         {
//            mesh.getIndicesBuffer().put(part.indices);
//         }
//         offset += meshPart.size;
//         meshParts.add(meshPart);
//      }
//      mesh.getIndicesBuffer().position(0);
//      for (MeshPart part : meshParts)
//         part.update();
//   }
//
//   private static void loadMaterials(Iterable<ModelMaterial> modelMaterials, TextureProvider textureProvider)
//   {
//      for (ModelMaterial mtl : modelMaterials)
//      {
//         this.materials.add(convertMaterial(mtl, textureProvider));
//      }
//   }
//
//   private static Material convertMaterial(ModelMaterial mtl, TextureProvider textureProvider)
//   {
//      Material result = new Material();
//      result.id = mtl.id;
//      if (mtl.ambient != null)
//         result.set(new ColorAttribute(ColorAttribute.Ambient, mtl.ambient));
//      if (mtl.diffuse != null)
//         result.set(new ColorAttribute(ColorAttribute.Diffuse, mtl.diffuse));
//      if (mtl.specular != null)
//         result.set(new ColorAttribute(ColorAttribute.Specular, mtl.specular));
//      if (mtl.emissive != null)
//         result.set(new ColorAttribute(ColorAttribute.Emissive, mtl.emissive));
//      if (mtl.reflection != null)
//         result.set(new ColorAttribute(ColorAttribute.Reflection, mtl.reflection));
//      if (mtl.shininess > 0f)
//         result.set(new FloatAttribute(FloatAttribute.Shininess, mtl.shininess));
//      if (mtl.opacity != 1.f)
//         result.set(new BlendingAttribute(GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA, mtl.opacity));
//
//      ObjectMap<String, Texture> textures = new ObjectMap<String, Texture>();
//
//      // FIXME uvScaling/uvTranslation totally ignored
//      if (mtl.textures != null)
//      {
//         for (ModelTexture tex : mtl.textures)
//         {
//            Texture texture;
//            if (textures.containsKey(tex.fileName))
//            {
//               texture = textures.get(tex.fileName);
//            }
//            else
//            {
//               texture = textureProvider.load(tex.fileName);
//               textures.put(tex.fileName, texture);
//               disposables.add(texture);
//            }
//
//            TextureDescriptor descriptor = new TextureDescriptor(texture);
//            descriptor.minFilter = texture.getMinFilter();
//            descriptor.magFilter = texture.getMagFilter();
//            descriptor.uWrap = texture.getUWrap();
//            descriptor.vWrap = texture.getVWrap();
//
//            float offsetU = tex.uvTranslation == null ? 0f : tex.uvTranslation.x;
//            float offsetV = tex.uvTranslation == null ? 0f : tex.uvTranslation.y;
//            float scaleU = tex.uvScaling == null ? 1f : tex.uvScaling.x;
//            float scaleV = tex.uvScaling == null ? 1f : tex.uvScaling.y;
//
//            switch (tex.usage)
//            {
//               case ModelTexture.USAGE_DIFFUSE:
//                  result.set(new TextureAttribute(TextureAttribute.Diffuse, descriptor, offsetU, offsetV, scaleU, scaleV));
//                  break;
//               case ModelTexture.USAGE_SPECULAR:
//                  result.set(new TextureAttribute(TextureAttribute.Specular, descriptor, offsetU, offsetV, scaleU, scaleV));
//                  break;
//               case ModelTexture.USAGE_BUMP:
//                  result.set(new TextureAttribute(TextureAttribute.Bump, descriptor, offsetU, offsetV, scaleU, scaleV));
//                  break;
//               case ModelTexture.USAGE_NORMAL:
//                  result.set(new TextureAttribute(TextureAttribute.Normal, descriptor, offsetU, offsetV, scaleU, scaleV));
//                  break;
//               case ModelTexture.USAGE_AMBIENT:
//                  result.set(new TextureAttribute(TextureAttribute.Ambient, descriptor, offsetU, offsetV, scaleU, scaleV));
//                  break;
//               case ModelTexture.USAGE_EMISSIVE:
//                  result.set(new TextureAttribute(TextureAttribute.Emissive, descriptor, offsetU, offsetV, scaleU, scaleV));
//                  break;
//               case ModelTexture.USAGE_REFLECTION:
//                  result.set(new TextureAttribute(TextureAttribute.Reflection, descriptor, offsetU, offsetV, scaleU, scaleV));
//                  break;
//            }
//         }
//      }
//
//      return result;
//   }
//
//   private static void loadNodes(Iterable<ModelNode> modelNodes)
//   {
//      nodePartBones.clear();
//      for (ModelNode node : modelNodes)
//      {
//         nodes.add(loadNode(node));
//      }
//      for (ObjectMap.Entry<NodePart, ArrayMap<String, Matrix4>> e : nodePartBones.entries())
//      {
//         if (e.key.invBoneBindTransforms == null)
//            e.key.invBoneBindTransforms = new ArrayMap<Node, Matrix4>(Node.class, Matrix4.class);
//         e.key.invBoneBindTransforms.clear();
//         for (ObjectMap.Entry<String, Matrix4> b : e.value.entries())
//            e.key.invBoneBindTransforms.put(getNode(b.key), new Matrix4(b.value).inv());
//      }
//   }
//
//   private static Node loadNode(ModelNode modelNode)
//   {
//      Node node = new Node();
//      node.id = modelNode.id;
//
//      if (modelNode.translation != null)
//         node.translation.set(modelNode.translation);
//      if (modelNode.rotation != null)
//         node.rotation.set(modelNode.rotation);
//      if (modelNode.scale != null)
//         node.scale.set(modelNode.scale);
//      // FIXME create temporary maps for faster lookup?
//      if (modelNode.parts != null)
//      {
//         for (ModelNodePart modelNodePart : modelNode.parts)
//         {
//            MeshPart meshPart = null;
//            Material meshMaterial = null;
//
//            if (modelNodePart.meshPartId != null)
//            {
//               for (MeshPart part : meshParts)
//               {
//                  if (modelNodePart.meshPartId.equals(part.id))
//                  {
//                     meshPart = part;
//                     break;
//                  }
//               }
//            }
//
//            if (modelNodePart.materialId != null)
//            {
//               for (Material material : materials)
//               {
//                  if (modelNodePart.materialId.equals(material.id))
//                  {
//                     meshMaterial = material;
//                     break;
//                  }
//               }
//            }
//
//            if (meshPart == null || meshMaterial == null)
//               throw new GdxRuntimeException("Invalid node: " + node.id);
//
//            NodePart nodePart = new NodePart();
//            nodePart.meshPart = meshPart;
//            nodePart.material = meshMaterial;
//            node.parts.add(nodePart);
//            if (modelNodePart.bones != null)
//               nodePartBones.put(nodePart, modelNodePart.bones);
//         }
//      }
//
//      if (modelNode.children != null)
//      {
//         for (ModelNode child : modelNode.children)
//         {
//            node.addChild(loadNode(child));
//         }
//      }
//
//      return node;
//   }
}
