package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXModelViewer
{
   public RDXModelViewer(String modelFileName)
   {
      RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();
            sceneManager.addCoordinateFrame(1.0);

            Model model = RDXModelLoader.load(modelFileName);
            sceneManager.addModelInstance(new ModelInstance(model));
         }

         @Override
         public void render()
         {
            sceneManager.render();
         }
      }, "Model Viewer", 1100, 800);
   }
}
