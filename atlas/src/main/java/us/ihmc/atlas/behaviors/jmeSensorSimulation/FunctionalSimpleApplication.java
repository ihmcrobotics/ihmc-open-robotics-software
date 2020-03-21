package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.app.SimpleApplication;

public class FunctionalSimpleApplication extends SimpleApplication
{
   private Runnable simpleInitApp;
   private Runnable initialize;

   /**
    * Nothing in super.
    */
   @Override
   public void simpleInitApp()
   {
      if (simpleInitApp != null) simpleInitApp.run();
   }

   @Override
   public void initialize()
   {
      super.initialize();

      if (initialize != null) initialize.run();
   }

   public void setSimpleInitApp(Runnable simpleInitApp)
   {
      this.simpleInitApp = simpleInitApp;
   }

   public void setInitialize(Runnable initialize)
   {
      this.initialize = initialize;
   }
}
