package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import controller_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI.ExternalForceEstimationToolboxConfigurationCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.ArrayList;
import java.util.List;

public class ExternalForceEstimationToolboxModule extends ToolboxModule
{
   public static final int UPDATE_PERIOD_MILLIS = 60;
   private static final double defaultTimeWithoutInputsBeforeSleep = 60.0;

   private final ExternalForceEstimationToolboxController forceEstimationToolboxController;

   public ExternalForceEstimationToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, UPDATE_PERIOD_MILLIS, pubSubImplementation);
      this.forceEstimationToolboxController = new ExternalForceEstimationToolboxController(robotModel, fullRobotModel, commandInputManager, statusOutputManager, yoGraphicsListRegistry, UPDATE_PERIOD_MILLIS, registry);
      timeWithoutInputsBeforeGoingToSleep.set(defaultTimeWithoutInputsBeforeSleep);
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator, s ->
      {
         if(forceEstimationToolboxController != null)
            forceEstimationToolboxController.updateRobotConfigurationData(s.takeNextData());
      });

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotDesiredConfigurationData.class, controllerPubGenerator, s ->
      {
         if(forceEstimationToolboxController != null)
            forceEstimationToolboxController.updateRobotDesiredConfigurationData(s.takeNextData());
      });
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return forceEstimationToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return getSupportedCommands();
   }

   public static List<Class<? extends Command<?, ?>>> getSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(ExternalForceEstimationToolboxConfigurationCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return getSupportedStatuses();
   }

   public static List<Class<? extends Settable<?>>> getSupportedStatuses()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(ExternalForceEstimationOutputStatus.class);
      return status;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.EXTERNAL_FORCE_ESTIMATION_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.EXTERNAL_FORCE_ESTIMATION_TOOLBOX, ROS2TopicQualifier.INPUT);
   }
}
