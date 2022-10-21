package us.ihmc.rdx.tools.assimp;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMaterial;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMesh;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.TextureProvider;
import org.lwjgl.PointerBuffer;
import org.lwjgl.assimp.*;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class RDXAssimpModelLoader
{
   private final String basePath;
   private final AssimpResourceImporter assimpResourceImporter = new AssimpResourceImporter();
   private final String modelFilePath;

   public RDXAssimpModelLoader(String modelFilePath)
   {
      this.modelFilePath = modelFilePath;

      FileHandle fileHandle = Gdx.files.internal(modelFilePath);
      basePath = fileHandle.parent().path();
   }

   public Model load()
   {
      TextureProvider.FileTextureProvider textureProvider = new TextureProvider.FileTextureProvider();
      ModelData modelData = loadModelData();
      Model model = new Model(modelData, textureProvider);
      return model;
   }

   public ModelData loadModelData()
   {
      AIPropertyStore assimpPropertyStore = Assimp.aiCreatePropertyStore();

      int postProcessingSteps = 0; // none

      /** libGDX reads UVs flipped from assimp default */
      postProcessingSteps += Assimp.aiProcess_FlipUVs;

      /** libGDX needs triangles */
      postProcessingSteps += Assimp.aiProcess_Triangulate;

      /** libGDX has limits in MeshBuilder.
       *  Not sure if there is a triangle limit.
       */
      Assimp.aiSetImportPropertyInteger(assimpPropertyStore, Assimp.AI_CONFIG_PP_SLM_VERTEX_LIMIT, MeshBuilder.MAX_VERTICES);
      //      Assimp.aiSetImportPropertyInteger(assimpPropertyStore, Assimp.AI_CONFIG_PP_SLM_TRIANGLE_LIMIT, MeshBuilder.MAX_VERTICES);
      postProcessingSteps += Assimp.aiProcess_SplitLargeMeshes;

      //      postProcessingSteps += Assimp.aiProcess_OptimizeGraph;
      //      postProcessingSteps += Assimp.aiProcess_OptimizeMeshes;
      //      postProcessingSteps += Assimp.aiProcess_JoinIdenticalVertices;

      AIScene assimpScene = assimpResourceImporter.importScene(modelFilePath, postProcessingSteps, assimpPropertyStore);

      ModelData modelData = new ModelData();
      modelData.id = "";

      // meshes
      int numberOfMeshes = assimpScene.mNumMeshes();
      LogTools.debug("Number of meshes: {}", numberOfMeshes);
      PointerBuffer meshesPointerBuffer = assimpScene.mMeshes();
      modelData.meshes.ensureCapacity(numberOfMeshes);
      ArrayList<RDXAssimpMeshLoader> rdxAssimpMeshLoaders = new ArrayList<>();
      for (int i = 0; i < numberOfMeshes; i++)
      {
         AIMesh assimpMesh = new AIMesh(MemoryUtil.memByteBuffer(meshesPointerBuffer.get(i), AIMesh.SIZEOF));
         RDXAssimpMeshLoader rdxAssimpMeshLoader = new RDXAssimpMeshLoader(assimpMesh);
         ModelMesh modelMesh = rdxAssimpMeshLoader.load();
         rdxAssimpMeshLoaders.add(rdxAssimpMeshLoader);
         modelData.meshes.add(modelMesh);
      }

      // materials
      int numberOfMaterials = assimpScene.mNumMaterials();
      LogTools.debug("Number of materials: {}", numberOfMaterials);
      ArrayList<RDXAssimpMaterialLoader> RDXAssimpMaterialLoaders = new ArrayList<>();
      if (numberOfMaterials > 0)
      {
         modelData.materials.ensureCapacity(numberOfMaterials);
         PointerBuffer materialsPointerBuffer = assimpScene.mMaterials();
         for (int i = 0; i < numberOfMaterials; i++)
         {
            AIMaterial assimpMaterial = new AIMaterial(MemoryUtil.memByteBuffer(materialsPointerBuffer.get(i), AIMaterial.SIZEOF));
            RDXAssimpMaterialLoader RDXAssimpMaterialLoader = new RDXAssimpMaterialLoader(assimpMaterial, basePath);
            RDXAssimpMaterialLoaders.add(RDXAssimpMaterialLoader);
            ModelMaterial modelMaterial = RDXAssimpMaterialLoader.load();
            modelData.materials.add(modelMaterial);
         }
      }

      // nodes
      AINode assimpRootNode = assimpScene.mRootNode();
      modelData.nodes.ensureCapacity(1);
      modelData.nodes.add(new RDXAssimpNodeLoader(rdxAssimpMeshLoaders, RDXAssimpMaterialLoaders).load(assimpRootNode));

      return modelData;
   }
}
