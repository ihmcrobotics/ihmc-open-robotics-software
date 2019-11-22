package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.IOException;
import java.util.List;

public class FootstepPlanningToolboxModule extends ToolboxModule
{
   private final FootstepPlanningToolboxController footstepPlanningToolboxController;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   public FootstepPlanningToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer)
   {
      this(drcRobotModel, modelProvider, startYoVariableServer, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public FootstepPlanningToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer,
                                        DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(drcRobotModel.getSimpleRobotName(), drcRobotModel.createFullRobotModel(), modelProvider, startYoVariableServer, pubSubImplementation);
      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      footstepPlanningToolboxController = new FootstepPlanningToolboxController(drcRobotModel.getContactPointParameters(),
                                                                                drcRobotModel.getFootstepPlannerParameters(),
                                                                                drcRobotModel.getVisibilityGraphsParameters(), statusOutputManager, registry,
                                                                                yoGraphicsListRegistry);
      footstepPlanningToolboxController.setTextToSpeechPublisher(textToSpeechPublisher);
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlanningRequestPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlannerParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processFootstepPlannerParameters(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, VisibilityGraphsParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processVisibilityGraphsParameters(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PlanningStatisticsRequestMessage.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processPlanningStatisticsRequest());
      textToSpeechPublisher = ROS2Tools.createPublisher(realtimeRos2Node, TextToSpeechPacket.class, ROS2Tools::generateDefaultTopicName);
   }
   
   @Override
   public void sleep()
   {
      footstepPlanningToolboxController.finishUp();
      super.sleep();
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return footstepPlanningToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return FootstepPlannerCommunicationProperties.getSupportedCommands();
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return FootstepPlannerCommunicationProperties.getSupportedStatusMessages();
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName);
   }
}
