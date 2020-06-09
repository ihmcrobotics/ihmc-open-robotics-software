package us.ihmc.atlas.sensors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;

public class AltasPerceptionSuiteLauncher extends Application
{
   private AtlasPerceptionSuite perceptionSuite;

   private LIDARBasedEnvironmentAwarenessUI reaUI;
   private SLAMBasedEnvironmentAwarenessUI slamUI;
   private PlanarSegmentationUI planarSegmentationUI;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      perceptionSuite = new AtlasPerceptionSuite();

      slamUI = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage);

      Stage secondStage = new Stage();
      planarSegmentationUI = PlanarSegmentationUI.creatIntraprocessUI(secondStage);

      Stage thirdStage = new Stage();
      reaUI = LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(thirdStage);

      slamUI.show();
      planarSegmentationUI.show();
      reaUI.show();

      perceptionSuite.start();
   }

   @Override
   public void stop() throws Exception
   {
      perceptionSuite.stop();

      reaUI.stop();
      slamUI.stop();
      planarSegmentationUI.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
