package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.gdx.vr.GDXVRApplication;

public class GDXVRDemo
{
   public GDXVRDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new PrivateGDXApplication(), getClass());
   }

   class PrivateGDXApplication extends Lwjgl3ApplicationAdapter
   {
      GDX3DApplication base = new GDX3DApplication();
      GDXVRApplication vr = new GDXVRApplication(base::renderVRCamera);

      @Override
      public void create()
      {
         base.create();
         vr.create();

         base.addCoordinateFrame(0.3);
         base.addModelInstance(new BoxesDemoModel().newInstance());
      }

      @Override
      public void render()
      {
         for (ModelInstance modelInstance : vr.getModelInstances())
         {
            base.addModelInstance(modelInstance);
         }

         base.render();
         vr.render();
      }

      @Override
      public void dispose()
      {
         vr.dispose();
         base.dispose();
      }
   }

   public static void main(String[] args)
   {
      new GDXVRDemo();
   }
}
