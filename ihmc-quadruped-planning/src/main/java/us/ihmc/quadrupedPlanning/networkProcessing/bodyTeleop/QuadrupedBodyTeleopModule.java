package us.ihmc.quadrupedPlanning.networkProcessing.bodyTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class QuadrupedBodyTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 1;

   private final QuadrupedBodyTeleopController bodyTeleopController;

   public QuadrupedBodyTeleopModule(FullQuadrupedRobotModelFactory modelFactory, LogModelProvider modelProvider, boolean startYoVariableServer, DomainFactory.PubSubImplementation pubSubImplementation)
         throws IOException
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, startYoVariableServer, updatePeriodMilliseconds,
            pubSubImplementation);

      QuadrupedRobotModelProviderNode robotModelProvider = new QuadrupedRobotModelProviderNode(robotName, realtimeRos2Node, modelFactory);

      bodyTeleopController = new QuadrupedBodyTeleopController(commandInputManager, statusOutputManager, robotModelProvider, registry);
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, getPublisherTopicNameGenerator(),
                                           s -> bodyTeleopController.processTimestamp(s.takeNextData().getTimestamp()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, getPublisherTopicNameGenerator(),
                                           s -> bodyTeleopController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, getPublisherTopicNameGenerator(),
                                           s -> bodyTeleopController.processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, getPublisherTopicNameGenerator(),
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
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();
      statusMessages.add(HighLevelStateMessage.class);
      statusMessages.add(HighLevelStateChangeStatusMessage.class);
      statusMessages.add(QuadrupedSteppingStateChangeMessage.class);
      statusMessages.add(RobotConfigurationData.class);

      return statusMessages;
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
