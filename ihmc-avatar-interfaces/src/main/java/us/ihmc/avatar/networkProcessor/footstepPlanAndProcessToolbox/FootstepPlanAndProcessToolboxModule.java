package us.ihmc.avatar.networkProcessor.footstepPlanAndProcessToolbox;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingToolboxController;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.postProcessing.AreaSplitFractionPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.PositionSplitFractionPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.SwingOverRegionsPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.YoVariablesForFootstepPostProcessingParameters;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.List;

public class FootstepPlanAndProcessToolboxModule extends ToolboxModule
{
   private final FootstepPlanningToolboxController footstepPlanningToolboxController;
   private final FootstepPlanPostProcessingToolboxController postProcessingToolboxController;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   public FootstepPlanAndProcessToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer)
   {
      this(drcRobotModel, modelProvider, startYoVariableServer, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public FootstepPlanAndProcessToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer,
                                              DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(drcRobotModel.getSimpleRobotName(), drcRobotModel.createFullRobotModel(), modelProvider, startYoVariableServer, pubSubImplementation);
      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      footstepPlanningToolboxController = new FootstepPlanningToolboxController(drcRobotModel.getContactPointParameters(),
                                                                                drcRobotModel.getFootstepPlannerParameters(),
                                                                                drcRobotModel.getVisibilityGraphsParameters(), statusOutputManager, registry,
                                                                                yoGraphicsListRegistry);
      footstepPlanningToolboxController.setTextToSpeechPublisher(textToSpeechPublisher);

      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      FootstepPostProcessingParametersBasics postProcessingParameters = drcRobotModel.getFootstepPostProcessingParameters();
      if (postProcessingParameters == null)
         postProcessingParameters = new DefaultFootstepPostProcessingParameters();

      postProcessingToolboxController = new FootstepPlanPostProcessingToolboxController(postProcessingParameters,
                                                                                        drcRobotModel.getWalkingControllerParameters(),
                                                                                        drcRobotModel.getContactPointParameters(),
                                                                                        drcRobotModel.getCapturePointPlannerParameters(), statusOutputManager,
                                                                                        registry, yoGraphicsListRegistry);

      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlanningRequestPacket.class, getSubscriberTopicNameGenerator(), s -> {
         footstepPlanningToolboxController.processRequest(s.takeNextData());
         footstepPlanningToolboxController.updateInternal();
      });
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlannerParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processFootstepPlannerParameters(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, VisibilityGraphsParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processVisibilityGraphsParameters(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PlanningStatisticsRequestMessage.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningToolboxController.processPlanningStatisticsRequest());

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPostProcessingPacket.class, getSubscriberTopicNameGenerator(), s -> {
         postProcessingToolboxController.processPostProcessingPacket(s.takeNextData());
         postProcessingToolboxController.updateInternal();
      });
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPostProcessingParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> postProcessingToolboxController.processFootstepPostProcessingParameters(s.takeNextData()));

      textToSpeechPublisher = ROS2Tools.createPublisher(realtimeRos2Node, TextToSpeechPacket.class, ROS2Tools::generateDefaultTopicName);
   }

   @Override
   public void sleep()
   {
      footstepPlanningToolboxController.finishUp();
      postProcessingToolboxController.finishUp();
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
