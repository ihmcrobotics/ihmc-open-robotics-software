package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXModelViewer
{
   public RDXModelViewer(String modelFileName)
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
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
