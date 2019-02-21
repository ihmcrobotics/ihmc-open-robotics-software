package us.ihmc.quadrupedCommunication.networkProcessing.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class QuadrupedFootstepPlanningModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 50;

   private final QuadrupedFootstepPlanningController footstepPlanningController;

   public QuadrupedFootstepPlanningModule(FullQuadrupedRobotModelFactory modelFactory, FootstepPlannerParameters defaultFootstepPlannerParameters,
                                          QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                          LogModelProvider modelProvider, boolean startYoVariableServer,
                                          DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), defaultFootstepPlannerParameters, defaultXGaitSettings,
           pointFootSnapperParameters, modelProvider, startYoVariableServer, pubSubImplementation);
   }

   public QuadrupedFootstepPlanningModule(String name, FullQuadrupedRobotModel fulRobotModel, FootstepPlannerParameters defaultFootstepPlannerParameters,
                                          QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                          LogModelProvider modelProvider, boolean startYoVariableServer,
                                          DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(name, fulRobotModel, modelProvider, startYoVariableServer, updatePeriodMilliseconds, pubSubImplementation);


      footstepPlanningController = new QuadrupedFootstepPlanningController(defaultXGaitSettings, new DefaultVisibilityGraphParameters(), defaultFootstepPlannerParameters,
                                                                           pointFootSnapperParameters, outputManager, robotDataReceiver, registry,
                                                                           yoGraphicsListRegistry, updatePeriodMilliseconds);
      new DefaultParameterReader().readParametersInRegistry(registry);
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator,
                                           s -> footstepPlanningController.processRobotTimestamp(s.takeNextData().getTimestamp()));
//      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> footstepPlanningController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> footstepPlanningController.processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> footstepPlanningController.processSteppingStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedGroundPlaneMessage.class, controllerPubGenerator,
                                           s -> footstepPlanningController.processGroundPlaneMessage(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedFootstepPlanningRequestPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningController.processFootstepPlanningRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedXGaitSettingsPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningController.processXGaitSettingsPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PlanarRegionsListMessage.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepPlanningController.processPlanarRegionsListMessage(s.takeNextData()));
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

      messages.put(QuadrupedFootstepPlanningToolboxOutputStatus.class, getPublisherTopicNameGenerator());
      messages.put(QuadrupedBodyOrientationMessage.class, getPublisherTopicNameGenerator());
      messages.put(BodyPathPlanMessage.class, getPublisherTopicNameGenerator());
      messages.put(QuadrupedFootstepPlannerParametersPacket.class, getPublisherTopicNameGenerator());
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
