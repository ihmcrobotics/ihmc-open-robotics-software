package us.ihmc.valkyrie.joystick;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickBasedSteppingMainUI;
import us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXController.SecondaryControlOption;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieJoystickBasedSteppingApplication extends Application
{
   private JoystickBasedSteppingMainUI ui;
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_valkyrie_xbox_joystick_control");

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      String robotTargetString = getParameters().getNamed().getOrDefault("robotTarget", "REAL_ROBOT");
      RobotTarget robotTarget = RobotTarget.valueOf(robotTargetString);
      PrintTools.info("-------------------------------------------------------------------");
      PrintTools.info("  -------- Loading parameters for RobotTarget: " + robotTarget + "  -------");
      PrintTools.info("-------------------------------------------------------------------");
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(robotTarget, false, "DEFAULT", null, false, true);
      String robotName = robotModel.getSimpleRobotName();
      ValkyriePunchMessenger kickAndPunchMessenger = new ValkyriePunchMessenger(robotName, ros2Node);

      ui = new JoystickBasedSteppingMainUI(robotName, primaryStage, ros2Node, robotModel, robotModel.getWalkingControllerParameters(), null,
                                           kickAndPunchMessenger, kickAndPunchMessenger);
      ui.setActiveSecondaryControlOption(SecondaryControlOption.PUNCH);
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
