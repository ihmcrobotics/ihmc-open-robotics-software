package us.ihmc.valkyrie.joystick;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickBasedSteppingMainUI;
import us.ihmc.commons.PrintTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieJoystickBasedSteppingApplication extends Application
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
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(robotTarget, false);
      ValkyrieKickAndPunchMessenger kickAndPunchMessenger = new ValkyrieKickAndPunchMessenger();

      ui = new JoystickBasedSteppingMainUI(primaryStage, robotModel, robotModel.getWalkingControllerParameters(), kickAndPunchMessenger, kickAndPunchMessenger,
                                           kickAndPunchMessenger);
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
