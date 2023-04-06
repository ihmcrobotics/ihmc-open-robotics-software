package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXAxisCircleDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXAxisBody axisBody;

   public RDXAxisCircleDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));

            axisBody = new RDXAxisBody(baseUI);
            baseUI.getPrimaryScene().addRenderableProvider(axisBody);
         }

         @Override
         public void render()
         {
            // 1st: let's draw line at point (x1, y1, z1) , (x2, y2, z2)
            axisBody.update();
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
      new RDXAxisCircleDemo();
   }
}
