package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;

public class FootstepPlanningToolboxModule extends ToolboxModule
{
   private final FootstepPlanningToolboxController footstepPlanningToolboxController;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   public FootstepPlanningToolboxModule(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullHumanoidRobotModel, LogModelProvider modelProvider,
                                        boolean startYoVariableServer) throws IOException
   {
      super(drcRobotModel.getSimpleRobotName(), fullHumanoidRobotModel, modelProvider, startYoVariableServer);
      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      footstepPlanningToolboxController = new FootstepPlanningToolboxController(drcRobotModel.getContactPointParameters(),
                                                                                drcRobotModel.getFootstepPlannerParameters(), statusOutputManager, registry,
                                                                                yoGraphicsListRegistry,
                                                                                Conversions.millisecondsToSeconds(DEFAULT_UPDATE_PERIOD_MILLISECONDS));
      footstepPlanningToolboxController.setTextToSpeechPublisher(textToSpeechPublisher);
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlanningRequestPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlannerParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processPlannerParameters(s.takeNextData()));
      textToSpeechPublisher = ROS2Tools.createPublisher(realtimeRos2Node, TextToSpeechPacket.class, ROS2Tools::generateDefaultTopicName);
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return footstepPlanningToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(FootstepPlanningToolboxOutputStatus.class);
      return status;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2TopicQualifier.INPUT);
   }
}
