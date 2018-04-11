package us.ihmc.atlas.joystickBasedStepping;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;

public class AtlasJoystickBasedSteppingApplication extends Application
{
   private JoystickBasedSteppingMainUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);

      ui = new JoystickBasedSteppingMainUI(primaryStage, atlasRobotModel, atlasRobotModel.getWalkingControllerParameters());
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();
      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch();
   }
}
