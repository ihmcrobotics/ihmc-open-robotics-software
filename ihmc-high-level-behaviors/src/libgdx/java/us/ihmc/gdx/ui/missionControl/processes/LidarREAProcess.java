package us.ihmc.gdx.ui.missionControl.processes;

import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.robotEnvironmentAwareness.LidarBasedREAStandaloneLauncher;

public class LidarREAProcess extends RestartableMissionControlProcess
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
