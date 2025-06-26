package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Attribute;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.model.Animation;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodeAnimation;
import com.badlogic.gdx.graphics.g3d.model.NodeKeyframe;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.ArrayMap;
import com.badlogic.gdx.utils.ObjectMap;

import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

/**
 * Deep copies a {@link Model}.
 */
public class LibGDXModelCopier
{
   public static Model deepCopy(Model model)
   {
      Model modelCopy = new Model();

      // Create mappings to maintain references between original and copied objects
      ObjectMap<Material, Material> materialMap = new ObjectMap<>();
      ObjectMap<Mesh, Mesh> meshMap = new ObjectMap<>();
      ObjectMap<MeshPart, MeshPart> meshPartMap = new ObjectMap<>();
      ObjectMap<Node, Node> nodeMap = new ObjectMap<>();

      // Copy meshes first (needed by mesh parts)
      for (Mesh originalMesh : model.meshes)
      {
         Mesh copiedMesh = copyMesh(originalMesh);
         modelCopy.meshes.add(copiedMesh);
         modelCopy.manageDisposable(copiedMesh);
         meshMap.put(originalMesh, copiedMesh);
      }

      // Copy materials
      for (Material originalMaterial : model.materials)
      {
         Material copiedMaterial = copyMaterial(originalMaterial);
         modelCopy.materials.add(copiedMaterial);
         materialMap.put(originalMaterial, copiedMaterial);
      }

      // Copy mesh parts
      for (MeshPart originalMeshPart : model.meshParts)
      {
         MeshPart copiedMeshPart = copyMeshPart(originalMeshPart, meshMap);
         modelCopy.meshParts.add(copiedMeshPart);
         meshPartMap.put(originalMeshPart, copiedMeshPart);
      }

      // Copy nodes (recursive)
      for (Node originalNode : model.nodes)
      {
         Node copiedNode = copyNode(originalNode, materialMap, meshPartMap, nodeMap);
         modelCopy.nodes.add(copiedNode);
      }

      // Copy animations
      for (Animation originalAnimation : model.animations)
      {
         Animation copiedAnimation = copyAnimation(originalAnimation, nodeMap);
         modelCopy.animations.add(copiedAnimation);
      }

      // Calculate transforms for the copied model
      modelCopy.calculateTransforms();

      return modelCopy;
   }

   private static Mesh copyMesh(Mesh originalMesh)
   {
      VertexAttributes attributes = originalMesh.getVertexAttributes();
      int numVertices = originalMesh.getNumVertices();
      int numIndices = originalMesh.getNumIndices();

      Mesh copiedMesh = new Mesh(true, numVertices, numIndices, attributes);

      // Copy vertex data
      FloatBuffer originalVertices = originalMesh.getVerticesBuffer(false);
      FloatBuffer copiedVertices = copiedMesh.getVerticesBuffer(true);
      originalVertices.position(0);
      copiedVertices.clear();
      copiedVertices.put(originalVertices);
      originalVertices.position(0);
      copiedVertices.position(0);

      // Copy index data if present
      if (numIndices > 0)
      {
         ShortBuffer originalIndices = originalMesh.getIndicesBuffer(false);
         ShortBuffer copiedIndices = copiedMesh.getIndicesBuffer(true);
         originalIndices.position(0);
         copiedIndices.clear();
         copiedIndices.put(originalIndices);
         originalIndices.position(0);
         copiedIndices.position(0);
      }

      return copiedMesh;
   }

   private static Material copyMaterial(Material originalMaterial)
   {
      Material copiedMaterial = new Material();
      copiedMaterial.id = originalMaterial.id;

      // Copy all attributes
      for (Attribute attribute : originalMaterial)
      {
         copiedMaterial.set(attribute.copy());
      }

      return copiedMaterial;
   }

   private static MeshPart copyMeshPart(MeshPart originalMeshPart, ObjectMap<Mesh, Mesh> meshMap)
   {
      MeshPart copiedMeshPart = new MeshPart();
      copiedMeshPart.id = originalMeshPart.id;
      copiedMeshPart.primitiveType = originalMeshPart.primitiveType;
      copiedMeshPart.offset = originalMeshPart.offset;
      copiedMeshPart.size = originalMeshPart.size;
      copiedMeshPart.mesh = meshMap.get(originalMeshPart.mesh);

      copiedMeshPart.update();

      return copiedMeshPart;
   }

   private static Node copyNode(Node originalNode,
                                ObjectMap<Material, Material> materialMap,
                                ObjectMap<MeshPart, MeshPart> meshPartMap,
                                ObjectMap<Node, Node> nodeMap)
   {
      Node copiedNode = new Node();
      copiedNode.id = originalNode.id;

      // Copy transform components
      copiedNode.translation.set(originalNode.translation);
      copiedNode.rotation.set(originalNode.rotation);
      copiedNode.scale.set(originalNode.scale);

      // Copy node parts
      for (NodePart originalNodePart : originalNode.parts)
      {
         NodePart copiedNodePart = copyNodePart(originalNodePart, materialMap, meshPartMap, nodeMap);
         copiedNode.parts.add(copiedNodePart);
      }

      // Add to mapping before copying children to handle circular references
      nodeMap.put(originalNode, copiedNode);

      // Copy children recursively
      for (Node originalChild : originalNode.getChildren())
      {
         Node copiedChild = nodeMap.get(originalChild);
         if (copiedChild == null)
         {
            copiedChild = copyNode(originalChild, materialMap, meshPartMap, nodeMap);
         }
         copiedNode.addChild(copiedChild);
      }

      return copiedNode;
   }

   private static NodePart copyNodePart(NodePart originalNodePart,
                                        ObjectMap<Material, Material> materialMap,
                                        ObjectMap<MeshPart, MeshPart> meshPartMap,
                                        ObjectMap<Node, Node> nodeMap)
   {
      NodePart copiedNodePart = new NodePart();
      copiedNodePart.meshPart = meshPartMap.get(originalNodePart.meshPart);
      copiedNodePart.material = materialMap.get(originalNodePart.material);

      // Copy bone transforms if present
      if (originalNodePart.invBoneBindTransforms != null)
      {
         copiedNodePart.invBoneBindTransforms = new ArrayMap<>(Node.class, Matrix4.class);
         for (ObjectMap.Entry<Node, Matrix4> entry : originalNodePart.invBoneBindTransforms.entries())
         {
            Node copiedBoneNode = nodeMap.get(entry.key);
            if (copiedBoneNode != null)
            {
               copiedNodePart.invBoneBindTransforms.put(copiedBoneNode, new Matrix4(entry.value));
            }
         }
      }

      return copiedNodePart;
   }

   private static Animation copyAnimation(Animation originalAnimation, ObjectMap<Node, Node> nodeMap)
   {
      Animation copiedAnimation = new Animation();
      copiedAnimation.id = originalAnimation.id;
      copiedAnimation.duration = originalAnimation.duration;

      // Copy node animations
      for (NodeAnimation originalNodeAnim : originalAnimation.nodeAnimations)
      {
         Node copiedNode = nodeMap.get(originalNodeAnim.node);
         if (copiedNode != null)
         {
            NodeAnimation copiedNodeAnim = copyNodeAnimation(originalNodeAnim, copiedNode);
            copiedAnimation.nodeAnimations.add(copiedNodeAnim);
         }
      }

      return copiedAnimation;
   }

   private static NodeAnimation copyNodeAnimation(NodeAnimation originalNodeAnim, Node copiedNode)
   {
      NodeAnimation copiedNodeAnim = new NodeAnimation();
      copiedNodeAnim.node = copiedNode;

      // Copy translation keyframes
      if (originalNodeAnim.translation != null)
      {
         copiedNodeAnim.translation = new Array<>();
         for (NodeKeyframe<Vector3> originalKeyframe : originalNodeAnim.translation)
         {
            NodeKeyframe<Vector3> copiedKeyframe = new NodeKeyframe<>(originalKeyframe.keytime, new Vector3(originalKeyframe.value));
            copiedNodeAnim.translation.add(copiedKeyframe);
         }
      }

      // Copy rotation keyframes
      if (originalNodeAnim.rotation != null)
      {
         copiedNodeAnim.rotation = new Array<>();
         for (NodeKeyframe<Quaternion> originalKeyframe : originalNodeAnim.rotation)
         {
            NodeKeyframe<Quaternion> copiedKeyframe = new NodeKeyframe<>(originalKeyframe.keytime, new Quaternion(originalKeyframe.value));
            copiedNodeAnim.rotation.add(copiedKeyframe);
         }
      }

      // Copy scaling keyframes
      if (originalNodeAnim.scaling != null)
      {
         copiedNodeAnim.scaling = new Array<>();
         for (NodeKeyframe<Vector3> originalKeyframe : originalNodeAnim.scaling)
         {
            NodeKeyframe<Vector3> copiedKeyframe = new NodeKeyframe<>(originalKeyframe.keytime, new Vector3(originalKeyframe.value));
            copiedNodeAnim.scaling.add(copiedKeyframe);
         }
      }

      return copiedNodeAnim;
   }
}
