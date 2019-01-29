package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

public class KinematicsPlanningToolboxModule extends ToolboxModule
{
   private final KinematicsPlanningToolboxController kinematicsPlanningToolboxController;

   public KinematicsPlanningToolboxModule(DRCRobotModel drcRobotModel, boolean startYoVariableServer) throws IOException
   {
      this(drcRobotModel, startYoVariableServer, PubSubImplementation.FAST_RTPS);
   }

   public KinematicsPlanningToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation) throws IOException
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer,
            DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);
      kinematicsPlanningToolboxController = new KinematicsPlanningToolboxController(robotModel, fullRobotModel, commandInputManager, statusOutputManager,
                                                                                    yoGraphicsListRegistry, registry);
      commandInputManager.registerConversionHelper(new KinematicsPlanningToolboxCommandConverter(fullRobotModel));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator, s -> {
         if (kinematicsPlanningToolboxController != null)
            kinematicsPlanningToolboxController.updateRobotConfigurationData(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, CapturabilityBasedStatus.class, controllerPubGenerator, s -> {
         if (kinematicsPlanningToolboxController != null)
            kinematicsPlanningToolboxController.updateCapturabilityBasedStatus(s.takeNextData());
      });
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return kinematicsPlanningToolboxController;
   }

   public static List<Class<? extends Settable<?>>> supportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(KinematicsPlanningToolboxOutputStatus.class);
      status.add(KinematicsToolboxOutputStatus.class);
      return status;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return supportedCommands();
   }

   static List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(KinematicsToolboxConfigurationCommand.class);
      commands.add(KinematicsPlanningToolboxCenterOfMassCommand.class);
      commands.add(KinematicsPlanningToolboxRigidBodyCommand.class);
      commands.add(KinematicsPlanningToolboxInputCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_PLANNING_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_PLANNING_TOOLBOX, ROS2TopicQualifier.INPUT);
   }
}
