package us.ihmc.valkyrie.joystick;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.fingers.ValkyrieFingerTrajectoryMessagePublisher;

public class ValkyrieJoystickBasedGraspingApplication extends ApplicationNoModule
{
   private JoystickBasedGraspingMainUI ui;
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "ihmc_valkyrie_xbox_joystick_control");

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      String robotTargetString = getParameters().getNamed().getOrDefault("robotTarget", "REAL_ROBOT");
      RobotTarget robotTarget = RobotTarget.valueOf(robotTargetString);
      LogTools.info("-------------------------------------------------------------------");
      LogTools.info("  -------- Loading parameters for RobotTarget: " + robotTarget + "  -------");
      LogTools.info("-------------------------------------------------------------------");
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(robotTarget, ValkyrieRobotVersion.DEFAULT);
      String robotName = robotModel.getSimpleRobotName();

      ROS2Topic inputTopic = ROS2Tools.getControllerInputTopic(robotName);

      ValkyrieFingerTrajectoryMessagePublisher handFingerTrajectoryMessagePublisher = new ValkyrieFingerTrajectoryMessagePublisher(ros2Node,
                                                                                                                                   inputTopic);

      ui = new JoystickBasedGraspingMainUI(robotName, primaryStage, ros2Node, robotModel, handFingerTrajectoryMessagePublisher);
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