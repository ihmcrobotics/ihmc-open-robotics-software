package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.FloatAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.Animation;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodeAnimation;
import com.badlogic.gdx.graphics.g3d.model.NodeKeyframe;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import com.badlogic.gdx.graphics.g3d.model.data.ModelAnimation;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMaterial;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMesh;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMeshPart;
import com.badlogic.gdx.graphics.g3d.model.data.ModelNode;
import com.badlogic.gdx.graphics.g3d.model.data.ModelNodeAnimation;
import com.badlogic.gdx.graphics.g3d.model.data.ModelNodeKeyframe;
import com.badlogic.gdx.graphics.g3d.model.data.ModelNodePart;
import com.badlogic.gdx.graphics.g3d.model.data.ModelTexture;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;

import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;

public class ModelToModelDataConverter
{
   public final ModelData modelData = new ModelData();

   public ModelToModelDataConverter(Model model)
   {
      // Convert materials
      convertMaterials(model);

      // Convert meshes
      convertMeshes(model);

      // Convert nodes
      convertNodes(model);

      // Convert animations
      convertAnimations(model);

      // Set version (default values)
      modelData.version[0] = 0;
      modelData.version[1] = 1;
   }

   private void convertMaterials(Model model)
   {
      for (Material material : model.materials)
      {
         ModelMaterial modelMaterial = new ModelMaterial();
         modelMaterial.id = material.id;

         // Convert color attributes
         if (material.has(ColorAttribute.Ambient))
         {
            ColorAttribute attr = (ColorAttribute) material.get(ColorAttribute.Ambient);
            modelMaterial.ambient = new Color(attr.color);
         }

         if (material.has(ColorAttribute.Diffuse))
         {
            ColorAttribute attr = (ColorAttribute) material.get(ColorAttribute.Diffuse);
            modelMaterial.diffuse = new Color(attr.color);
         }

         if (material.has(ColorAttribute.Specular))
         {
            ColorAttribute attr = (ColorAttribute) material.get(ColorAttribute.Specular);
            modelMaterial.specular = new Color(attr.color);
         }

         if (material.has(ColorAttribute.Emissive))
         {
            ColorAttribute attr = (ColorAttribute) material.get(ColorAttribute.Emissive);
            modelMaterial.emissive = new Color(attr.color);
         }

         if (material.has(ColorAttribute.Reflection))
         {
            ColorAttribute attr = (ColorAttribute) material.get(ColorAttribute.Reflection);
            modelMaterial.reflection = new Color(attr.color);
         }

         // Convert float attributes
         if (material.has(FloatAttribute.Shininess))
         {
            FloatAttribute attr = (FloatAttribute) material.get(FloatAttribute.Shininess);
            modelMaterial.shininess = attr.value;
         }

         // Convert blending attribute
         if (material.has(BlendingAttribute.Type))
         {
            BlendingAttribute attr = (BlendingAttribute) material.get(BlendingAttribute.Type);
            modelMaterial.opacity = attr.opacity;
         }

         // Convert textures
         modelMaterial.textures = new Array<ModelTexture>();
         convertTextureAttribute(material, TextureAttribute.Diffuse, ModelTexture.USAGE_DIFFUSE, modelMaterial);
         convertTextureAttribute(material, TextureAttribute.Specular, ModelTexture.USAGE_SPECULAR, modelMaterial);
         convertTextureAttribute(material, TextureAttribute.Bump, ModelTexture.USAGE_BUMP, modelMaterial);
         convertTextureAttribute(material, TextureAttribute.Normal, ModelTexture.USAGE_NORMAL, modelMaterial);
         convertTextureAttribute(material, TextureAttribute.Ambient, ModelTexture.USAGE_AMBIENT, modelMaterial);
         convertTextureAttribute(material, TextureAttribute.Emissive, ModelTexture.USAGE_EMISSIVE, modelMaterial);
         convertTextureAttribute(material, TextureAttribute.Reflection, ModelTexture.USAGE_REFLECTION, modelMaterial);

         modelData.materials.add(modelMaterial);
      }
   }

   private void convertTextureAttribute(Material material, long attributeType, int usage, ModelMaterial modelMaterial)
   {
      if (material.has(attributeType))
      {
         TextureAttribute attr = (TextureAttribute) material.get(attributeType);
         ModelTexture modelTexture = new ModelTexture();
         modelTexture.usage = usage;
         modelTexture.fileName = attr.textureDescription.texture.toString(); // This might need adjustment based on your texture loading
         modelTexture.uvTranslation = new Vector2(attr.offsetU, attr.offsetV);
         modelTexture.uvScaling = new Vector2(attr.scaleU, attr.scaleV);
         modelMaterial.textures.add(modelTexture);
      }
   }

   private void convertMeshes(Model model)
   {
      for (Mesh mesh : model.meshes)
      {
         ModelMesh modelMesh = new ModelMesh();
         modelMesh.id = "mesh_" + model.meshes.indexOf(mesh, true); // Generate ID if not available

         // Convert vertex attributes
         ArrayList<VertexAttribute> attributesList = new ArrayList<>();
         for (int i = 0; i < mesh.getVertexAttributes().size(); i++)
            attributesList.add(mesh.getVertexAttributes().get(i));
         modelMesh.attributes = attributesList.toArray(new VertexAttribute[0]);

         // Convert vertices
         int vertexSize = mesh.getVertexSize() / 4; // size in floats
         int numVertices = mesh.getNumVertices();
         modelMesh.vertices = new float[numVertices * vertexSize];

         FloatBuffer verticesBuffer = mesh.getVerticesBuffer(false);
         verticesBuffer.position(0);
         verticesBuffer.get(modelMesh.vertices);

         // Convert mesh parts
         ArrayList<ModelMeshPart> partsList = new ArrayList<>();
         for (MeshPart meshPart : model.meshParts)
         {
            if (meshPart.mesh == mesh)
            {
               ModelMeshPart modelMeshPart = new ModelMeshPart();
               modelMeshPart.id = meshPart.id;
               modelMeshPart.primitiveType = meshPart.primitiveType;

               // Extract indices for this mesh part
               if (mesh.getNumIndices() > 0)
               {
                  ShortBuffer indicesBuffer = mesh.getIndicesBuffer(false);
                  modelMeshPart.indices = new short[meshPart.size];

                  indicesBuffer.position(meshPart.offset);
                  indicesBuffer.get(modelMeshPart.indices, 0, meshPart.size);
               }
               else
               {
                  // No indices, create sequential indices
                  modelMeshPart.indices = new short[meshPart.size];
                  for (int i = 0; i < meshPart.size; i++)
                  {
                     modelMeshPart.indices[i] = (short) (meshPart.offset + i);
                  }
               }

               partsList.add(modelMeshPart);
            }
         }
         modelMesh.parts = partsList.toArray(new ModelMeshPart[0]);

         modelData.meshes.add(modelMesh);
      }
   }

   private void convertNodes(Model model)
   {
      for (Node node : model.nodes)
      {
         ModelNode modelNode = convertNode(node, model);
         modelData.nodes.add(modelNode);
      }
   }

   private ModelNode convertNode(Node node, Model model)
   {
      ModelNode modelNode = new ModelNode();
      modelNode.id = node.id;

      // Convert transformation
      modelNode.translation = new Vector3(node.translation);
      modelNode.rotation = new Quaternion(node.rotation);
      modelNode.scale = new Vector3(node.scale);

      // Convert node parts
      if (node.parts.size > 0)
      {
         ArrayList<ModelNodePart> partsList = new ArrayList<>();
         for (NodePart nodePart : node.parts)
         {
            ModelNodePart modelNodePart = new ModelNodePart();
            modelNodePart.meshPartId = nodePart.meshPart.id;
            modelNodePart.materialId = nodePart.material.id;

            // Convert bone bindings if present
//            if (nodePart.invBoneBindTransforms != null && nodePart.invBoneBindTransforms.size > 0)
//            {
//               modelNodePart.bones = new ArrayMap<String, com.badlogic.gdx.math.Matrix4>();
//               for (ArrayMap.Entry<Node, com.badlogic.gdx.math.Matrix4> entry : nodePart.invBoneBindTransforms.entries())
//               {
//                  modelNodePart.bones.put(entry.key.id, new com.badlogic.gdx.math.Matrix4(entry.value).inv());
//               }
//            }

            partsList.add(modelNodePart);
         }
         modelNode.parts = partsList.toArray(new ModelNodePart[0]);
      }

      // Convert children recursively
      if (node.getChildCount() > 0)
      {
         ArrayList<ModelNode> childrenList = new ArrayList<>();
         for (Node child : node.getChildren())
         {
            childrenList.add(convertNode(child, model));
         }
         modelNode.children = childrenList.toArray(new ModelNode[0]);
      }

      return modelNode;
   }

   private void convertAnimations(Model model)
   {
      for (Animation animation : model.animations)
      {
         ModelAnimation modelAnimation = new ModelAnimation();
         modelAnimation.id = animation.id;

         modelAnimation.nodeAnimations = new Array<ModelNodeAnimation>();
         for (NodeAnimation nodeAnimation : animation.nodeAnimations)
         {
            ModelNodeAnimation modelNodeAnimation = new ModelNodeAnimation();
            modelNodeAnimation.nodeId = nodeAnimation.node.id;

            // Convert translation keyframes
            if (nodeAnimation.translation != null && nodeAnimation.translation.size > 0)
            {
               modelNodeAnimation.translation = new Array<ModelNodeKeyframe<Vector3>>();
               for (NodeKeyframe<Vector3> keyframe : nodeAnimation.translation)
               {
                  ModelNodeKeyframe<Vector3> modelKeyframe = new ModelNodeKeyframe<Vector3>();
                  modelKeyframe.keytime = keyframe.keytime;
                  modelKeyframe.value = new Vector3(keyframe.value);
                  modelNodeAnimation.translation.add(modelKeyframe);
               }
            }

            // Convert rotation keyframes
            if (nodeAnimation.rotation != null && nodeAnimation.rotation.size > 0)
            {
               modelNodeAnimation.rotation = new Array<ModelNodeKeyframe<Quaternion>>();
               for (NodeKeyframe<Quaternion> keyframe : nodeAnimation.rotation)
               {
                  ModelNodeKeyframe<Quaternion> modelKeyframe = new ModelNodeKeyframe<Quaternion>();
                  modelKeyframe.keytime = keyframe.keytime;
                  modelKeyframe.value = new Quaternion(keyframe.value);
                  modelNodeAnimation.rotation.add(modelKeyframe);
               }
            }

            // Convert scaling keyframes
            if (nodeAnimation.scaling != null && nodeAnimation.scaling.size > 0)
            {
               modelNodeAnimation.scaling = new Array<ModelNodeKeyframe<Vector3>>();
               for (NodeKeyframe<Vector3> keyframe : nodeAnimation.scaling)
               {
                  ModelNodeKeyframe<Vector3> modelKeyframe = new ModelNodeKeyframe<Vector3>();
                  modelKeyframe.keytime = keyframe.keytime;
                  modelKeyframe.value = new Vector3(keyframe.value);
                  modelNodeAnimation.scaling.add(modelKeyframe);
               }
            }

            modelAnimation.nodeAnimations.add(modelNodeAnimation);
         }

         modelData.animations.add(modelAnimation);
      }
   }

   public ModelData getModelData()
   {
      return modelData;
   }
}