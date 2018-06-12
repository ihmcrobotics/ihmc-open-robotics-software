package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatusPubSubType;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.ros2.RealtimeRos2Node;

public class KinematicsToolboxModule extends ToolboxModule
{
   private final HumanoidKinematicsToolboxController kinematicsToolBoxController;

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer) throws IOException
   {
      super(robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer);
      kinematicsToolBoxController = new HumanoidKinematicsToolboxController(commandInputManager, statusOutputManager, fullRobotModel, yoGraphicsListRegistry,
                                                                            registry);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(fullRobotModel));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, new RobotConfigurationDataPubSubType(), "/ihmc/robot_configuration_data",
                                           s -> kinematicsToolBoxController.updateRobotConfigurationData(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, new CapturabilityBasedStatusPubSubType(), "/ihmc/capturability_based_status",
                                           s -> kinematicsToolBoxController.updateCapturabilityBasedStatus(s.takeNextData()));
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
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return new MessageTopicNameGenerator()
      {
         private final String prefix = TOOLBOX_ROS_TOPIC_PREFIX + "/ik/input";

         @Override
         public String generateTopicName(Class<? extends Settable<?>> messageType)
         {
            return ROS2Tools.appendTypeToTopicName(prefix, messageType);
         }
      };
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return new MessageTopicNameGenerator()
      {
         private final String prefix = TOOLBOX_ROS_TOPIC_PREFIX + "/ik/output";

         @Override
         public String generateTopicName(Class<? extends Settable<?>> messageType)
         {
            return ROS2Tools.appendTypeToTopicName(prefix, messageType);
         }
      };
   }
}
