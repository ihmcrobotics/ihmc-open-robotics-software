package us.ihmc.jme;

import com.jme3.app.SimpleApplication;

public class FunctionalSimpleApplication extends SimpleApplication
{
   private Runnable simpleInitApp;
   private Runnable initialize;
   private JMEFloatConsumer simpleUpdate;

   public interface JMEFloatConsumer
   {
      void accept(float timePerFrame);
   }

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

   @Override
   public void simpleUpdate(float tpf)
   {
      // nothing in super
      if (simpleUpdate != null) simpleUpdate.accept(tpf);
   }

   public void setSimpleInitApp(Runnable simpleInitApp)
   {
      this.simpleInitApp = simpleInitApp;
   }

   public void setInitialize(Runnable initialize)
   {
      this.initialize = initialize;
   }

   public void setSimpleUpdate(JMEFloatConsumer simpleUpdate)
   {
      this.simpleUpdate = simpleUpdate;
   }
}