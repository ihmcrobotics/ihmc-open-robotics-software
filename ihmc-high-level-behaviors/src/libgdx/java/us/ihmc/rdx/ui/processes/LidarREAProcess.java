package us.ihmc.rdx.ui.processes;

import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.robotEnvironmentAwareness.LidarBasedREAStandaloneLauncher;

public class LidarREAProcess extends RestartableProcess
{
   private LidarBasedREAStandaloneLauncher lidarBasedREAStandaloneLauncher;

   @Override
   protected void startInternal()
   {
      lidarBasedREAStandaloneLauncher = new LidarBasedREAStandaloneLauncher();
      JavaFXApplicationCreator.buildJavaFXApplication(primaryStage ->
      {
         try
         {
            lidarBasedREAStandaloneLauncher.start(primaryStage);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      });
   }

   @Override
   protected void stopInternal()
   {
      try
      {
         lidarBasedREAStandaloneLauncher.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public String getName()
   {
      return "Lidar REA";
   }
}
