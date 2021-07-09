package us.ihmc.gdx;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXApplicationCreator;

public class GDX3DTextRenderingDemo
{
   public GDX3DTextRenderingDemo()
   {
      GDX3DSceneManager sceneManager = new GDX3DSceneManager();
      //GDXTextRenderer textRenderer = new GDXTextRenderer();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());

            //textRenderer.create();
            //sceneManager.addRenderableProvider(textRenderer);
         }

         @Override
         public void render()
         {
            //textRenderer.setTextAtPosition(new Point3D(0.5, 0.5, 0.5), "Text rendering demo (very cool)");

            GDX3DSceneTools.glClearGray();
            //textRenderer.update();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDX3DTextRenderingDemo();
   }
}
