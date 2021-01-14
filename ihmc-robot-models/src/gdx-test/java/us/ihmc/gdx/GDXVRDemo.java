package us.ihmc.gdx;

import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.vr.GDXVRApplication;

public class GDXVRDemo
{
   private GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private GDXVRApplication vr = new GDXVRApplication();

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
         vr.create();

         sceneManager.addCoordinateFrame(0.3);
         sceneManager.addModelInstance(new BoxesDemoModel().newInstance());
         sceneManager.addRenderableProvider(vr);
      }

      @Override
      public void render()
      {
         sceneManager.glClearGray();
         sceneManager.render();
         vr.render(sceneManager);
      }

      @Override
      public void dispose()
      {
         vr.dispose();
         sceneManager.dispose();
      }
   }

   public static void main(String[] args)
   {
      new GDXVRDemo();
   }
}
