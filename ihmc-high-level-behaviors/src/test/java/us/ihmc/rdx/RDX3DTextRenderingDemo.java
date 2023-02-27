package us.ihmc.rdx;

import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;

public class RDX3DTextRenderingDemo
{
   public RDX3DTextRenderingDemo()
   {
      RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private RDX3DSituatedText text;

         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());

            text = new RDX3DSituatedText("test");
            sceneManager.addRenderableProvider(text);
         }

         @Override
         public void render()
         {
            RDX3DSceneTools.glClearGray();
            sceneManager.setViewportBoundsToWindow();

            text.getModelInstance().transform.rotate(0, 0, 1, 1);

            sceneManager.render();
         }
      }, "RDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new RDX3DTextRenderingDemo();
   }
}
