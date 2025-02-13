package us.ihmc.rdx.demo;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.rdx.gdx.RDXLwjgl3ApplicationAdapter;
import us.ihmc.rdx.model.RDXModelInstance;
import us.ihmc.rdx.model.RDXModelLoader;
import us.ihmc.rdx.RDXBaseUI;

public class RDXModelViewer
{
   public RDXModelViewer(String modelFileName)
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new RDXLwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(1.0);

            Model model = RDXModelLoader.load(modelFileName);
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
}
