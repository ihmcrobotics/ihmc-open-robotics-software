package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.rdx.RDXBaseUI;
import us.ihmc.rdx.gdx.RDXLwjgl3ApplicationAdapter;
import us.ihmc.rdx.scene.RDXSceneLevel;
import us.ihmc.rdx.model.RDXModelInstance;
import us.ihmc.rdx.model.RDXModelLoader;

/**
 * Use this to check on environment object models as they look in engine.
 */
public class RDXEnvironmentObjectViewer
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   public RDXEnvironmentObjectViewer()
   {
      baseUI.launchRDXApplication(new RDXLwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);

            // Swap out this string to load different models
            Model model = RDXModelLoader.load("environmentObjects/ousterSensor/Ouster.g3dj");
            baseUI.getPrimaryScene().addModelInstance(new RDXModelInstance(model));
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

   public static void main(String[] args)
   {
      new RDXEnvironmentObjectViewer();
   }
}
