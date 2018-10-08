package us.ihmc.footstepPlanning.ui;

import com.sun.javafx.application.PlatformImpl;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;

public class ApplicationRunner
{

   public static void runApplication(Application launcher)
   {
      PlatformImpl.startup(() -> {
         Platform.runLater(new Runnable()
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
         });
      });
      PlatformImpl.setImplicitExit(false);
   }
}
