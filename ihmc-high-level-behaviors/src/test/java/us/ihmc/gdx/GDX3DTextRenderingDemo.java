package us.ihmc.gdx;

import us.ihmc.gdx.sceneManager.GDX3DBareBonesScene;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXApplicationCreator;

public class GDX3DTextRenderingDemo
{
   public GDX3DTextRenderingDemo()
   {
      GDX3DBareBonesScene sceneManager = new GDX3DBareBonesScene();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private GDX3DSituatedText text;

         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());

            text = new GDX3DSituatedText("test");
            sceneManager.addRenderableProvider(text);
         }

         @Override
         public void render()
         {
            GDX3DSceneTools.glClearGray();
            sceneManager.setViewportBoundsToWindow();

            text.getModelInstance().transform.rotate(0, 0, 1, 1);

            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDX3DTextRenderingDemo();
   }
}
