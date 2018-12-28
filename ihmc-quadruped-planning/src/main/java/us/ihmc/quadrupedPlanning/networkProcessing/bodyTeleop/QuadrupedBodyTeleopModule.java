package us.ihmc.quadrupedPlanning.networkProcessing.bodyTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class QuadrupedBodyTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 1;

   private final QuadrupedBodyTeleopController bodyTeleopController;

   public QuadrupedBodyTeleopModule(FullQuadrupedRobotModelFactory modelFactory, LogModelProvider modelProvider,
                                    DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, false, updatePeriodMilliseconds,
            pubSubImplementation);

      QuadrupedRobotModelProviderNode robotModelProvider = new QuadrupedRobotModelProviderNode(robotName, realtimeRos2Node, modelFactory);

      bodyTeleopController = new QuadrupedBodyTeleopController(outputManager, robotModelProvider, registry);
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator,
                                           s -> bodyTeleopController.processTimestamp(s.takeNextData().getTimestamp()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> bodyTeleopController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> bodyTeleopController.processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> bodyTeleopController.processSteppingStateChangeMessage(s.takeNextData()));
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return bodyTeleopController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return null;
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

   public void setDesiredBodyPose(double x, double y, double yaw, double pitch, double roll, double time)
   {
      bodyTeleopController.setDesiredBodyPose(x, y, yaw, pitch, roll, time);
   }
}
