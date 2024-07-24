package us.ihmc.rdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import net.mgsx.gltf.loaders.gltf.GLTFLoader;
import net.mgsx.gltf.scene3d.scene.SceneAsset;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXGLTFDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXGLTFDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            FileHandle fileHandle = Gdx.files.internal("models/BoomBox.gltf");
            SceneAsset sceneAsset = new GLTFLoader().load(fileHandle, true);
            ModelInstance modelInstance = new ModelInstance(sceneAsset.scene.model);
            modelInstance.transform.setToRotationRad(1.0f, 0.0f, 0.0f, (float) Math.PI / 2.0f);
            modelInstance.transform.translate(0.2f, 0.2f, 0.2f);
            modelInstance.transform.scale(20.0f, 20.0f, 20.0f);
            baseUI.getPrimaryScene().addModelInstance(modelInstance);

            baseUI.getImGuiPanelManager().addPanel("Settings", RDXGLTFDemo.this::renderImGuiWidgets);

         }

         @Override
         public void render()
         {
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

   }

   public static void main(String[] args)
   {
      new RDXGLTFDemo();
   }
}
