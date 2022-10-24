package us.ihmc.rdx.tools.assimp;

import com.badlogic.gdx.graphics.g3d.model.data.ModelNode;
import com.badlogic.gdx.graphics.g3d.model.data.ModelNodePart;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import org.lwjgl.assimp.AIMesh;
import org.lwjgl.assimp.AINode;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.euclid.matrix.LinearTransform3D;

import java.util.ArrayList;

public class RDXAssimpNodeLoader
{
   private final ArrayList<RDXAssimpMeshLoader> rdxAssimpMeshLoaders;
   private final ArrayList<RDXAssimpMaterialLoader> rdxAssimpMaterialLoaders;

   public RDXAssimpNodeLoader(ArrayList<RDXAssimpMeshLoader> rdxAssimpMeshLoaders, ArrayList<RDXAssimpMaterialLoader> rdxAssimpMaterialLoaders)
   {
      this.rdxAssimpMeshLoaders = rdxAssimpMeshLoaders;
      this.rdxAssimpMaterialLoaders = rdxAssimpMaterialLoaders;
   }

   public ModelNode load(AINode assimpRootNode)
   {
      return loadRecursively(assimpRootNode);
   }

   private ModelNode loadRecursively(AINode assimpNode)
   {
      ModelNode modelNode = new ModelNode();
      modelNode.id = assimpNode.mName().dataString();

      modelNode.translation = new Vector3();
      modelNode.translation.x = assimpNode.mTransformation().a4();
      modelNode.translation.y = assimpNode.mTransformation().b4();
      modelNode.translation.z = assimpNode.mTransformation().c4();

      LinearTransform3D linearTransform = new LinearTransform3D(assimpNode.mTransformation().a1(),
                                                                assimpNode.mTransformation().a2(),
                                                                assimpNode.mTransformation().a3(),
                                                                assimpNode.mTransformation().b1(),
                                                                assimpNode.mTransformation().b2(),
                                                                assimpNode.mTransformation().b3(),
                                                                assimpNode.mTransformation().c1(),
                                                                assimpNode.mTransformation().c2(),
                                                                assimpNode.mTransformation().c3());

      us.ihmc.euclid.tuple4D.Quaternion euclidQuaternion = new us.ihmc.euclid.tuple4D.Quaternion(linearTransform.getAsQuaternion());
      modelNode.rotation = new Quaternion();
      modelNode.rotation.x = euclidQuaternion.getX32();
      modelNode.rotation.y = euclidQuaternion.getY32();
      modelNode.rotation.z = euclidQuaternion.getZ32();
      modelNode.rotation.w = euclidQuaternion.getS32();

      modelNode.scale = null; // TODO: This okay?

      int numberOfMeshes = assimpNode.mNumMeshes();
      if (numberOfMeshes > 0)
      {
         modelNode.parts = new ModelNodePart[numberOfMeshes];
         for (int i = 0; i < numberOfMeshes; i++)
         {
            ModelNodePart modelNodePart = new ModelNodePart();
            int assimpMeshIndex = assimpNode.mMeshes().get(i);

            RDXAssimpMeshLoader rdxAssimpMeshLoader = rdxAssimpMeshLoaders.get(assimpMeshIndex);
            AIMesh assimpMesh = rdxAssimpMeshLoader.getAssimpMesh();
            int assimpMaterialIndex = assimpMesh.mMaterialIndex();
            modelNodePart.meshPartId = rdxAssimpMeshLoader.getModelMesh().id;
            modelNodePart.materialId = rdxAssimpMaterialLoaders.get(assimpMaterialIndex).getModelMaterial().id;
            modelNode.parts[i] = modelNodePart;
         }
      }

      int numberOfChildren = assimpNode.mNumChildren();
      if (numberOfChildren > 0)
      {
         modelNode.children = new ModelNode[numberOfChildren];
         for (int i = 0; i < numberOfChildren; i++)
         {
            modelNode.children[i] = loadRecursively(new AINode(MemoryUtil.memByteBuffer(assimpNode.mChildren().get(i), AINode.SIZEOF)));
         }
      }
      return modelNode;
   }
}
