package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelLoader;

/**
 * Use this to check on environment object models as they look in engine.
 */
public class RDXEnvironmentObjectViewer
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   public RDXEnvironmentObjectViewer()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);

            // Swap out this string to load different models
            Model model = RDXModelLoader.load("environmentObjects/ousterSensor/Ouster.g3dj");
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(model));
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
