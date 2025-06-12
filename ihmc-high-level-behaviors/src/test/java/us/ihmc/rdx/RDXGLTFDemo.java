package us.ihmc.rdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImFloat;
import net.mgsx.gltf.loaders.gltf.GLTFLoader;
import net.mgsx.gltf.scene3d.scene.SceneAsset;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.LibGDXModelCopier;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelInstanceScaler;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXGLTFDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXModelInstanceScaler scaler;
   private ModelInstance modelInstance2;
//   private float currentScale = 10.0f;
   private ImFloat currentScale = new ImFloat(10.0f);

   public RDXGLTFDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            Model load = RDXModelLoader.load("environmentObjects/flatGround/FlatGround.g3dj");
            load = LibGDXModelCopier.deepCopy(load);
            baseUI.getPrimaryScene().addModelInstance(new RDXModelInstance(load));

            FileHandle fileHandle = Gdx.files.internal("models/BoomBox.gltf");
            SceneAsset sceneAsset = new GLTFLoader().load(fileHandle, true);
            //            ModelInstance modelInstance = new ModelInstance(LibGDXModelCopier.deepCopy(sceneAsset.scene.model));
            ////            ModelInstance modelInstance = new ModelInstance(RDXModelLoader.load("models/BoomBox.gltf"));
            //            modelInstance.transform.setToRotationRad(1.0f, 0.0f, 0.0f, (float) Math.PI / 2.0f);
            //            modelInstance.transform.translate(0.2f, 0.2f, 0.2f);
            //            modelInstance.transform.scale(20.0f, 20.0f, 20.0f);
            //            baseUI.getPrimaryScene().addModelInstance(modelInstance);

            scaler = new RDXModelInstanceScaler(sceneAsset.scene.model);

            //            scaler.scale(10.5f);
            //            ModelInstance modelInstance2 = scaler.getModelInstance();
            modelInstance2 = new ModelInstance(scaler.getScaledDeepCopy(10.0));
            //            ModelInstance modelInstance = new ModelInstance(RDXModelLoader.load("models/BoomBox.gltf"));
//            modelInstance2.transform.setToRotationRad(1.0f, 0.0f, 0.0f, (float) Math.PI / 2.0f);
//            modelInstance2.transform.translate(1.2f, 0.2f, 0.2f);
//            float scale = currentScale.get();
//            modelInstance2.transform.scale(scale, scale, scale);
            baseUI.getPrimaryScene().addModelInstance(modelInstance2);
            //            baseUI.getPrimaryScene().addRenderableProvider(scaler::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Settings", RDXGLTFDemo.this::renderImGuiWidgets);
         }

         int i = 0;

         @Override
         public void render()
         {
            //            i++;
            //
            //            if (i == 5)
            //               scaler.scale(20.0f);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }


   private void renderImGuiWidgets()
   {
      if (ImGui.sliderFloat(labels.get("Scale"), currentScale.getData(), 1.0f, 50.0f))
      {
         modelInstance2.transform.setToScaling(currentScale.get(), currentScale.get(), currentScale.get());
      }
   }

   public static void main(String[] args)
   {
      new RDXGLTFDemo();
   }
}
