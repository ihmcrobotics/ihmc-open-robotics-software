package us.ihmc.javafx;

import com.sun.javafx.application.PlatformImpl;
import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;

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
}
