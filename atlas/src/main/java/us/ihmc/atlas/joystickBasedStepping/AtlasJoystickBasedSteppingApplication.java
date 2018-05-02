package us.ihmc.atlas.joystickBasedStepping;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.PrintTools;

public class AtlasJoystickBasedSteppingApplication extends Application
{
   private JoystickBasedSteppingMainUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      String robotTargetString = getParameters().getNamed().getOrDefault("robotTarget", "REAL_ROBOT");
      RobotTarget robotTarget = RobotTarget.valueOf(robotTargetString);
      PrintTools.info("-------------------------------------------------------------------");
      PrintTools.info("  -------- Loading parameters for RobotTarget: " + robotTarget + "  -------");
      PrintTools.info("-------------------------------------------------------------------");
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, robotTarget, false);
      AtlasKickAndPunchMessenger atlasKickAndPunchMessenger = new AtlasKickAndPunchMessenger();

      ui = new JoystickBasedSteppingMainUI(primaryStage, atlasRobotModel, atlasRobotModel.getWalkingControllerParameters(), atlasKickAndPunchMessenger,
                                           atlasKickAndPunchMessenger);
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();
      ui.stop();

      Platform.exit();
   }

   /**
    * 
    * @param args should either be {@code --robotTarget=SCS} or {@code --robotTarget=REAL_ROBOT}.
    */
   public static void main(String[] args)
   {
      launch(args);
   }
}
