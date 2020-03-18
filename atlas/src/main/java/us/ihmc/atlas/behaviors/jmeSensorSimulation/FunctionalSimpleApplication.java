package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.app.SimpleApplication;

public class FunctionalSimpleApplication extends SimpleApplication
{
   private Runnable simpleInitApp;

   public void setSimpleInitApp(Runnable simpleInitApp)
   {
      this.simpleInitApp = simpleInitApp;
   }

   /**
    * Nothing in super.
    */
   @Override
   public void simpleInitApp()
   {
      simpleInitApp.run();
   }
}
