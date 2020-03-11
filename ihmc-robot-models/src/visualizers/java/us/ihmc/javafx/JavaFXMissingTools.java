package us.ihmc.javafx;

import com.sun.javafx.application.PlatformImpl;
import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.stage.Stage;

import java.io.IOException;
import java.net.URL;

public class JavaFXMissingTools
{
   public static void runNFramesLater(int numberOfFramesToWait, Runnable runnable)
   {
      new AnimationTimer()
      {
         int counter = 0;

         @Override
         public void handle(long now)
         {
            if (counter++ > numberOfFramesToWait)
            {
               runnable.run();
               stop();
            }
         }
      }.start();
   }

   public static void runApplication(Application application)
   {
      runApplication(application, null);
   }

   public static void runApplication(Application application, Runnable initialize)
   {
      Runnable runnable = () ->
      {
         try
         {
            application.start(new Stage());
            if (initialize != null)
               initialize.run();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      };

      PlatformImpl.startup(() ->
                           {
                              Platform.runLater(runnable);
                           });
      PlatformImpl.setImplicitExit(false);
   }

   /**
    * - Controller class name must match the .fxml file name.
    * - Must not set fx:contoller in FXML file
    * - This method expects .fxml to be in same package path as controller class
    */
   public static <T> T loadFromFXML(Object controller)
   {
      FXMLLoader loader = new FXMLLoader();
      loader.setController(controller);
      URL resource = controller.getClass().getResource(controller.getClass().getSimpleName() + ".fxml");
      if (resource == null)
      {
         throw new RuntimeException("Cannot find " + controller.getClass().getSimpleName() + ".fxml, please check the resource path.");
      }
      loader.setLocation(resource);

      try
      {
         return loader.load();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
