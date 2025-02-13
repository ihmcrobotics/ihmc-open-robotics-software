package us.ihmc.rdx;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.gdx.RDXLwjgl3ApplicationAdapter;
import us.ihmc.rdx.graphics.RDX3DSituatedText;
import us.ihmc.rdx.graphics.RDX3DSituatedTextData;
import us.ihmc.rdx.demo.BoxesDemoModel;

public class RDX3DSituatedTextDemo
{
   public RDX3DSituatedTextDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new RDXLwjgl3ApplicationAdapter()
      {
         private RDX3DSituatedText text;
         private RDX3DSituatedText rapidlyChangingText;
         private RDX3DSituatedTextData previousTextData;
         private final Stopwatch stopwatch = new Stopwatch().start();

         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            text = new RDX3DSituatedText("test");
            baseUI.getPrimaryScene().addRenderableProvider(text);

            rapidlyChangingText = new RDX3DSituatedText("rapidly changing");
            baseUI.getPrimaryScene().addRenderableProvider(rapidlyChangingText);
         }

         @Override
         public void render()
         {
            text.getModelTransform().rotate(0.0f, 0.0f, 1.0f, 1.0f);

            if (previousTextData != null)
               previousTextData.dispose();

            previousTextData = rapidlyChangingText.setTextWithoutCache("Time: " + stopwatch.totalElapsed());
            rapidlyChangingText.getModelTransform().rotate(1.0f, 0.0f, 0.0f, 200.0f * (float) stopwatch.totalElapsed());

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
      new RDX3DSituatedTextDemo();
   }
}