package us.ihmc.quadrupedCommunication.networkProcessing.bodyTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;
import static us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor.bodyTeleopPort;

public class QuadrupedBodyTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 75;

   private final QuadrupedBodyTeleopController bodyTeleopController;

   public QuadrupedBodyTeleopModule(FullQuadrupedRobotModelFactory modelFactory, LogModelProvider modelProvider, boolean startYoVariableServer,
                                    boolean logYoVariables, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider,startYoVariableServer,
            new DataServerSettings(logYoVariables, true, bodyTeleopPort, "BodyTeleopModule"), updatePeriodMilliseconds, pubSubImplementation);

      bodyTeleopController = new QuadrupedBodyTeleopController(outputManager, robotDataReceiver, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator,
                                           s -> processTimestamp(s.takeNextData().getControllerTimestamp()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> processSteppingStateChangeMessage(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedTeleopDesiredPose.class, getSubscriberTopicNameGenerator(),
                                           s -> processTeleopDesiredPoseMessage(s.takeNextData()));
   }

   private void processTimestamp(long timestamp)
   {
      if (bodyTeleopController != null)
         bodyTeleopController.processTimestamp(timestamp);
   }

   private void setPaused(boolean paused)
   {
      if (bodyTeleopController != null)
         bodyTeleopController.setPaused(paused);
   }

   private void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      if (bodyTeleopController != null)
         bodyTeleopController.processHighLevelStateChangeMessage(message);
   }

   private void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      if (bodyTeleopController != null)
         bodyTeleopController.processSteppingStateChangeMessage(message);
   }

   private void processTeleopDesiredPoseMessage(QuadrupedTeleopDesiredPose message)
   {
      if (bodyTeleopController != null)
         bodyTeleopController.processTeleopDesiredPoseMessage(message);
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return bodyTeleopController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public Map<Class<? extends Settable<?>>, ROS2Tools.MessageTopicNameGenerator> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, ROS2Tools.MessageTopicNameGenerator> messages = new HashMap<>();

      ROS2Tools.MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      messages.put(QuadrupedBodyOrientationMessage.class, controllerSubGenerator);
      messages.put(QuadrupedBodyTrajectoryMessage.class, controllerSubGenerator);

      return messages;
   }

   @Override
   public ROS2Tools.MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.BODY_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public ROS2Tools.MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.BODY_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
   }

   @Override
   public void sleep()
   {
      bodyTeleopController.setPaused(true);

      super.sleep();
   }
}
