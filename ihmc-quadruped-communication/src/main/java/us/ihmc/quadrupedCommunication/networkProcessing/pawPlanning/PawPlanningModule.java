package us.ihmc.quadrupedCommunication.networkProcessing.pawPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;
import static us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor.footstepPlanningPort;

public class PawPlanningModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 50;

   private final PawPlanningController footstepPlanningController;

   public PawPlanningModule(FullQuadrupedRobotModelFactory modelFactory, PawStepPlannerParametersBasics defaultPawPlannerParameters,
                            QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                            LogModelProvider modelProvider, boolean startYoVariableServer, boolean logYoVariables,
                            DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), defaultPawPlannerParameters, defaultXGaitSettings,
           pointFootSnapperParameters, modelProvider, startYoVariableServer, logYoVariables, pubSubImplementation);
   }

   public PawPlanningModule(String name, FullQuadrupedRobotModel fulRobotModel, PawStepPlannerParametersBasics defaultPawPlannerParameters,
                            QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                            LogModelProvider modelProvider, boolean startYoVariableServer, boolean logYoVariables,
                            DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(name, fulRobotModel, modelProvider, startYoVariableServer, new DataServerSettings(logYoVariables, true, footstepPlanningPort,
                                                                                              "FootstepPlanningModule"), updatePeriodMilliseconds,
            pubSubImplementation);


      footstepPlanningController = new PawPlanningController(defaultXGaitSettings, new DefaultVisibilityGraphParameters(),
                                                             defaultPawPlannerParameters,
                                                             pointFootSnapperParameters, outputManager, robotDataReceiver, registry,
                                                             yoGraphicsListRegistry, updatePeriodMilliseconds);
      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator,
                                           s -> processRobotTimestamp(s.takeNextData().getMonotonicTime()));
//      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> footstepPlanningController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> processSteppingStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedGroundPlaneMessage.class, controllerPubGenerator,
                                           s -> processGroundPlaneMessage(s.takeNextData()));

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSupportPlanarRegionParametersMessage.class,
                                           ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.QUADRUPED_SUPPORT_REGION_PUBLISHER, ROS2TopicQualifier.INPUT),
                                           s -> processSupportRegionParameters(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PawStepPlanningRequestPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> processPawPlanningRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedXGaitSettingsPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> processXGaitSettingsPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PawStepPlannerParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> processFootstepPlannerParametersPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, VisibilityGraphsParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> processVisibilityGraphParametersPacket(s.takeNextData()));
   }

   private void processRobotTimestamp(long timestamp)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processRobotTimestamp(timestamp);
   }

   private void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processHighLevelStateChangeMessage(message);
   }

   private void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processSteppingStateChangeMessage(message);
   }

   private void processFootstepPlannerParametersPacket(PawStepPlannerParametersPacket packet)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processFootstepPlannerParametersPacket(packet);
   }

   private void processVisibilityGraphParametersPacket(VisibilityGraphsParametersPacket packet)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processVisibilityGraphParametersPacket(packet);
   }

   private void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processGroundPlaneMessage(message);
   }

   private void processSupportRegionParameters(QuadrupedSupportPlanarRegionParametersMessage message)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processSupportRegionParameters(message);
   }

   private void processPawPlanningRequest(PawStepPlanningRequestPacket packet)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processPawPlanningRequest(packet);
   }

   private void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processXGaitSettingsPacket(packet);
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return footstepPlanningController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public Map<Class<? extends Settable<?>>, MessageTopicNameGenerator> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, MessageTopicNameGenerator> messages = new HashMap<>();

      messages.put(PawStepPlanningToolboxOutputStatus.class, getPublisherTopicNameGenerator());
      messages.put(QuadrupedBodyOrientationMessage.class, getPublisherTopicNameGenerator());
      messages.put(BodyPathPlanMessage.class, getPublisherTopicNameGenerator());
      messages.put(PawStepPlannerParametersPacket.class, getPublisherTopicNameGenerator());
      messages.put(FootstepPlannerStatusMessage.class, getPublisherTopicNameGenerator());

      return messages;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
   }

   @Override
   public void sleep()
   {
//      footstepPlanningController.setPaused(true);

      super.sleep();
   }


}
