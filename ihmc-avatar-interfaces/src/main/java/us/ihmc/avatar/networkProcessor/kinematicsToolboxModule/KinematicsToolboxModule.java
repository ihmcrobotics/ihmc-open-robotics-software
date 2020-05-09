package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxOneDoFJointCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

public class KinematicsToolboxModule extends ToolboxModule
{
   private final HumanoidKinematicsToolboxController kinematicsToolBoxController;

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer,
            DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);
      kinematicsToolBoxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                            statusOutputManager,
                                                                            fullRobotModel,
                                                                            robotModel,
                                                                            Conversions.millisecondsToSeconds(updatePeriodMilliseconds),
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      kinematicsToolBoxController.setInitialRobotConfiguration(robotModel);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(fullRobotModel));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Topic controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, RobotConfigurationData.class, controllerOutputTopic, s ->
      {
         if (kinematicsToolBoxController != null)
            kinematicsToolBoxController.updateRobotConfigurationData(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, CapturabilityBasedStatus.class, controllerOutputTopic, s ->
      {
         if (kinematicsToolBoxController != null)
            kinematicsToolBoxController.updateCapturabilityBasedStatus(s.takeNextData());
      });
   }

   /**
    * This method defines the input API for this toolbox. You can find the corresponding messages to
    * these commands that can be sent over the network.
    * <p>
    * Do not forget that this toolbox will ignore any message with a destination different from
    * {@value KinematicsToolboxModule#PACKET_DESTINATION}.
    * </p>
    */
   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return supportedCommands();
   }

   public static List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(KinematicsToolboxCenterOfMassCommand.class);
      commands.add(KinematicsToolboxRigidBodyCommand.class);
      commands.add(KinematicsToolboxOneDoFJointCommand.class);
      commands.add(KinematicsToolboxConfigurationCommand.class);
      commands.add(HumanoidKinematicsToolboxConfigurationCommand.class);
      return commands;
   }

   /**
    * This method defines the output API for this toolbox. The message that this toolbox sends are
    * directed to the source the of the input messages.
    */
   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   public static List<Class<? extends Settable<?>>> supportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(KinematicsToolboxOutputStatus.class);
      return status;
   }

   @Override
   public KinematicsToolboxController getToolboxController()
   {
      return kinematicsToolBoxController;
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   @Override
   public ROS2Topic getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic getOutputTopic(String robotName)
   {
      return ROS2Tools.KINEMATICS_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic getInputTopic(String robotName)
   {
      return ROS2Tools.KINEMATICS_TOOLBOX.withRobot(robotName).withInput();
   }
}
