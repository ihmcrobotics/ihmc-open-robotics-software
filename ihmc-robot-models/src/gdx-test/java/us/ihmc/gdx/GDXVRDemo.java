package us.ihmc.gdx;

import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.vr.GDXVRManager;

public class GDXVRDemo
{
   private GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private GDXVRManager vrManager = new GDXVRManager();

   public GDXVRDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new PrivateGDXApplication(), getClass());
   }

   class PrivateGDXApplication extends Lwjgl3ApplicationAdapter
   {
      @Override
      public void create()
      {
         sceneManager.create();
         vrManager.create();

         sceneManager.addCoordinateFrame(0.3);
         sceneManager.addModelInstance(new BoxesDemoModel().newInstance());
         sceneManager.addRenderableProvider(vrManager);
      }

      @Override
      public void render()
      {
         sceneManager.glClearGray();
         sceneManager.render();
         vrManager.render(sceneManager);
      }

      @Override
      public void dispose()
      {
         vrManager.dispose();
         sceneManager.dispose();
      }
   }

   public static void main(String[] args)
   {
      new GDXVRDemo();
   }
}
