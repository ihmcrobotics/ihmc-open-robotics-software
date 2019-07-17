package us.ihmc.humanoidBehaviors.tools.perception;

import java.util.concurrent.CountDownLatch;

import javafx.application.Application;
import javafx.stage.Stage;

/**
 * This class allows for spinning up the JavaFX engine without having to have your code extend an Application.
 * To use, simply call createAJavaFXApplication(). Then make JavaFX windows pop up from non JavaFX Applications
 * by using JavaFX Stage, Scene, Group, etc. objects and then just doing Scene.show(); 
 * Unfortunately, you can only ever have one JavaFX "Application" 
 * running at the same time. This class makes it easy to ensure that you have one and only one.
 * See the test class for this class for an example of how to create and display a JavaFX scene.
 * 
 * @author JerryPratt
 *
 */
public class JavaFXApplicationCreator extends Application
{
   private static JavaFXApplicationCreator createdJavaFXApplicationSingleton;

   private static final CountDownLatch latch = new CountDownLatch(1);
   private static JavaFXApplicationCreator startUpTest = null;

   public JavaFXApplicationCreator()
   {
      setStartUpTest(this);
   }

   private void setStartUpTest(JavaFXApplicationCreator startUpTest0)
   {
      startUpTest = startUpTest0;
      latch.countDown();
   }

   private static JavaFXApplicationCreator waitForStartUpTest()
   {
      try
      {
         latch.await();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }

      return startUpTest;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
   }

   /**
    * Call this method to spin up the JavaFX engine. If it is already spun up, then
    * it will ignore the call. 
    * 
    * @return JavaFX Application that is being run. 
    */
   public static JavaFXApplicationCreator createAJavaFXApplication()
   {
      if (createdJavaFXApplicationSingleton != null)
         return createdJavaFXApplicationSingleton;

      new Thread()
      {
         @Override
         public void run()
         {
            javafx.application.Application.launch(JavaFXApplicationCreator.class);
         }
      }.start();

      createdJavaFXApplicationSingleton = JavaFXApplicationCreator.waitForStartUpTest();

      return createdJavaFXApplicationSingleton;
   }
}

