package us.ihmc.gdx.tools.assimp;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMaterial;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMesh;
import com.badlogic.gdx.graphics.g3d.utils.TextureProvider;
import org.lwjgl.PointerBuffer;
import org.lwjgl.assimp.AIMaterial;
import org.lwjgl.assimp.AIMesh;
import org.lwjgl.assimp.AINode;
import org.lwjgl.assimp.AIScene;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.log.LogTools;

public class GDXAssimpModelLoader
{
   private final AIScene assimpScene;
   private final String basePath;

   public GDXAssimpModelLoader(AIScene assimpScene, String basePath)
   {
      this.assimpScene = assimpScene;
      this.basePath = basePath;
   }

   public Model load()
   {
      ModelData modelData = new ModelData();
      modelData.id = "";

      // meshes
      int numberOfMeshes = assimpScene.mNumMeshes();
      LogTools.info("Number of meshes: {}", numberOfMeshes);
      PointerBuffer meshesPointerBuffer = assimpScene.mMeshes();
      modelData.meshes.ensureCapacity(numberOfMeshes);
      for (int i = 0; i < numberOfMeshes; i++)
      {
         AIMesh assimpMesh = new AIMesh(MemoryUtil.memByteBuffer(meshesPointerBuffer.get(i), AIMesh.SIZEOF));
         ModelMesh modelMesh = new GDXAssimpMeshLoader(assimpMesh).load(i);
         modelData.meshes.add(modelMesh);
      }

      // materials
      int numberOfMaterials = assimpScene.mNumMaterials();
      LogTools.info("Number of materials: {}", numberOfMaterials);
      if (numberOfMaterials > 0)
      {
         modelData.materials.ensureCapacity(numberOfMaterials);
         PointerBuffer materialsPointerBuffer = assimpScene.mMaterials();
         for (int i = 0; i < numberOfMaterials; i++)
         {
            AIMaterial assimpMaterial = new AIMaterial(MemoryUtil.memByteBuffer(materialsPointerBuffer.get(i), AIMaterial.SIZEOF));
            GDXAssimpMaterialLoader assimpMaterialLoader = new GDXAssimpMaterialLoader(assimpMaterial, basePath);
            ModelMaterial modelMaterial = assimpMaterialLoader.load();
            modelData.materials.add(modelMaterial);
         }
      }

      // nodes
      AINode assimpRootNode = assimpScene.mRootNode();
      modelData.nodes.ensureCapacity(1);
      modelData.nodes.add(new GDXAssimpNodeLoader(modelData.meshes, modelData.materials).load(assimpRootNode));

      TextureProvider.FileTextureProvider textureProvider = new TextureProvider.FileTextureProvider();
      Model model = new Model(modelData, textureProvider);
      return model;
   }
}
