package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

public class KinematicsStreamingToolboxModule extends ToolboxModule
{
   private static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 5;

   private final KinematicsStreamingToolboxController controller;
   private IHMCRealtimeROS2Publisher<WholeBodyTrajectoryMessage> outputPublisher;
   private final KinematicsStreamingToolboxMessageLogger logger;

   public KinematicsStreamingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer) throws IOException
   {
      this(robotModel, startYoVariableServer, PubSubImplementation.FAST_RTPS);
   }

   public KinematicsStreamingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
         throws IOException
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer,
            DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);

      setTimeWithoutInputsBeforeGoingToSleep(30.0);
      controller = new KinematicsStreamingToolboxController(commandInputManager,
                                                            statusOutputManager,
                                                            fullRobotModel,
                                                            robotModel,
                                                            robotModel.getControllerDT(),
                                                            Conversions.millisecondsToSeconds(updatePeriodMilliseconds),
                                                            yoGraphicsListRegistry,
                                                            registry);
      controller.setCollisionModel(robotModel.getHumanoidRobotKinematicsCollisionModel());
      controller.setOutputPublisher(outputPublisher::publish);
      commandInputManager.registerConversionHelper(new KinematicsStreamingToolboxCommandConverter(fullRobotModel));
      logger = new KinematicsStreamingToolboxMessageLogger(robotModel.getSimpleRobotName(), realtimeRos2Node);
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      MessageTopicNameGenerator controllerSubGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      outputPublisher = ROS2Tools.createPublisher(realtimeRos2Node, WholeBodyTrajectoryMessage.class, controllerSubGenerator);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator, s ->
      {
         if (controller != null)
            controller.updateRobotConfigurationData(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, CapturabilityBasedStatus.class, controllerPubGenerator, s ->
      {
         if (controller != null)
            controller.updateCapturabilityBasedStatus(s.takeNextData());
      });
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return controller;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return supportedCommands();
   }

   public static List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(KinematicsStreamingToolboxInputCommand.class);
      commands.add(KinematicsToolboxConfigurationCommand.class);
      return commands;
   }

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
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_STREAMING_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_STREAMING_TOOLBOX, ROS2TopicQualifier.INPUT);
   }

   @Override
   protected void startLogging()
   {
      logger.startLogging();
   }

   @Override
   protected void stopLogging()
   {
      logger.stopLogging();
   }
}
