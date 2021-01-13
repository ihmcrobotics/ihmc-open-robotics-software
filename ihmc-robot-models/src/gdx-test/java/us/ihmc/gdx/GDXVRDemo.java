package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.vr.GDXVRApplication;

public class GDXVRDemo
{
   private GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private GDXVRApplication vr = new GDXVRApplication(sceneManager::renderVRCamera);

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
      }

      @Override
      public void render()
      {
         for (ModelInstance modelInstance : vr.getModelInstances())
         {
            sceneManager.addModelInstance(modelInstance);
         }

         sceneManager.glClearGray();
         sceneManager.render();
         vr.render();
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
