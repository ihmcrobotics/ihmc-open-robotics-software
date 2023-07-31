package us.ihmc.rdx;

import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDX3DTextRenderingDemo
{
   public RDX3DTextRenderingDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private RDX3DSituatedText text;

         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            text = new RDX3DSituatedText("test");
            baseUI.getPrimaryScene().addRenderableProvider(text);
         }

         @Override
         public void render()
         {
            text.getModelTransform().rotate(0.0f, 0.0f, 1.0f, 1.0f);

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
      new RDX3DTextRenderingDemo();
   }
}
