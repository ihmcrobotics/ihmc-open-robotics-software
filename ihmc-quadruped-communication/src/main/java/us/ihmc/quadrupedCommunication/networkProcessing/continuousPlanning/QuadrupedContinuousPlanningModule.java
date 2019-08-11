package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
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
import static us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor.continuousPlanningPort;

public class QuadrupedContinuousPlanningModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 50;

   private final QuadrupedContinuousPlanningController continuousPlanningController;

   public QuadrupedContinuousPlanningModule(FullQuadrupedRobotModelFactory modelFactory, LogModelProvider modelProvider, boolean startYoVariableServer,
                                          boolean logYoVariables, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, startYoVariableServer, logYoVariables,
           pubSubImplementation);
   }

   public QuadrupedContinuousPlanningModule(String name, FullQuadrupedRobotModel fulRobotModel,
                                          LogModelProvider modelProvider, boolean startYoVariableServer, boolean logYoVariables,
                                          DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(name, fulRobotModel, modelProvider, startYoVariableServer, new DataServerSettings(logYoVariables, true, continuousPlanningPort,
                                                                                              "ContinuousPlanningModule"), updatePeriodMilliseconds,
            pubSubImplementation);

      continuousPlanningController = new QuadrupedContinuousPlanningController(outputManager, robotDataReceiver, registry, yoGraphicsListRegistry);

      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> processSteppingStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator,
                                           s -> processFootstepStatusMessage(s.takeNextData()));

      // status messages from the planner
      MessageTopicNameGenerator plannerPubGenerator = getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedFootstepPlanningToolboxOutputStatus.class, plannerPubGenerator,
                                           s -> processFootstepPlannerOutputMessage(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedContinuousPlanningRequestPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> processContinuousPlanningRequest(s.takeNextData()));
   }

   private void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processHighLevelStateChangeMessage(message);
   }

   private void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processSteppingStateChangeMessage(message);
   }

   private void processFootstepPlannerOutputMessage(QuadrupedFootstepPlanningToolboxOutputStatus footstepPlannerOutput)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processFootstepPlannerOutput(footstepPlannerOutput);
   }

   private void processContinuousPlanningRequest(QuadrupedContinuousPlanningRequestPacket planningRequestPacket)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processContinuousPlanningRequest(planningRequestPacket);
   }

   private void processFootstepStatusMessage(QuadrupedFootstepStatusMessage footstepStatusMessage)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processFootstepStatusMessage(footstepStatusMessage);
   }


   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return continuousPlanningController;
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

      return messages;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
   }
}
