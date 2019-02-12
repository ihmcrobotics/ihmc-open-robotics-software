package us.ihmc.quadrupedFootstepPlanning.ui;

import com.sun.javafx.application.PlatformImpl;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;

public class ApplicationRunner
{

   public static Runnable runApplication(Application launcher)
   {
      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               launcher.start(new Stage());
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      };

      PlatformImpl.startup(() -> {
         Platform.runLater(runnable);
      });
      PlatformImpl.setImplicitExit(false);

      return runnable;
   }
}
