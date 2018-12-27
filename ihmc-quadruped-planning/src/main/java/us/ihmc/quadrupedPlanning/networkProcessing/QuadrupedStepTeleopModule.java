package us.ihmc.quadrupedPlanning.networkProcessing;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.input.QuadrupedRobotModelProviderNode;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class QuadrupedStepTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 1;

   private final QuadrupedStepTeleopController stepTeleopController;

   public QuadrupedStepTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettings defaultXGaitSettings, double nominalHeight,
                                    LogModelProvider modelProvider, boolean startYoVariableServer, DomainFactory.PubSubImplementation pubSubImplementation)
         throws IOException
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, startYoVariableServer, updatePeriodMilliseconds,
            pubSubImplementation);

      QuadrupedRobotModelProviderNode robotModelProvider = new QuadrupedRobotModelProviderNode(robotName, realtimeRos2Node, modelFactory);

      stepTeleopController = new QuadrupedStepTeleopController(defaultXGaitSettings, nominalHeight, commandInputManager, statusOutputManager,
                                                               robotModelProvider, registry, yoGraphicsListRegistry, updatePeriodMilliseconds);
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, getPublisherTopicNameGenerator(),
                                           s -> stepTeleopController.processTimestamp(s.takeNextData().getTimestamp()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, getPublisherTopicNameGenerator(),
                                           s -> stepTeleopController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, getPublisherTopicNameGenerator(),
                                           s -> stepTeleopController.processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedBodyPathPlanMessage.class, getPublisherTopicNameGenerator(),
                                           s -> stepTeleopController.processBodyPathPlanMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedFootstepStatusMessage.class, getPublisherTopicNameGenerator(),
                                           s -> stepTeleopController.processFootstepStatusMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, getPublisherTopicNameGenerator(),
                                           s -> stepTeleopController.processSteppingStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedGroundPlaneMessage.class, getPublisherTopicNameGenerator(),
                                           s -> stepTeleopController.processGroundPlaneMessage(s.takeNextData()));
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return stepTeleopController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return null;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();
      statusMessages.add(HighLevelStateMessage.class);
      statusMessages.add(HighLevelStateChangeStatusMessage.class);
      statusMessages.add(QuadrupedBodyPathPlanMessage.class);
      statusMessages.add(QuadrupedFootstepStatusMessage.class);
      statusMessages.add(QuadrupedSteppingStateChangeMessage.class);
      statusMessages.add(QuadrupedGroundPlaneMessage.class);
      statusMessages.add(RobotConfigurationData.class);

      return statusMessages;
   }

   @Override
   public ROS2Tools.MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return QuadrupedStepTeleopCommunicationProperties.publisherTopicNameGenerator(robotName);
   }

   @Override
   public ROS2Tools.MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return QuadrupedStepTeleopCommunicationProperties.subscriberTopicNameGenerator(robotName);
   }

   @Override
   public void sleep()
   {
      stepTeleopController.setPaused(true);

      super.sleep();
   }
}
