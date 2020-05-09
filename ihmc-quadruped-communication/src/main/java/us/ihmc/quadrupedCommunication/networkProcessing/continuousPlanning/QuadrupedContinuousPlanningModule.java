package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor.continuousPlanningPort;

public class QuadrupedContinuousPlanningModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 50;

   private final QuadrupedContinuousPlanningController continuousPlanningController;

   public QuadrupedContinuousPlanningModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                            LogModelProvider modelProvider, boolean startYoVariableServer,
                                          boolean logYoVariables, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), defaultXGaitSettings, modelProvider, startYoVariableServer,
           logYoVariables, pubSubImplementation);
   }

   public QuadrupedContinuousPlanningModule(String name, FullQuadrupedRobotModel fulRobotModel, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                          LogModelProvider modelProvider, boolean startYoVariableServer, boolean logYoVariables,
                                          DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(name, fulRobotModel, modelProvider, startYoVariableServer, new DataServerSettings(logYoVariables, true, continuousPlanningPort,
                                                                                              "ContinuousPlanningModule"), updatePeriodMilliseconds,
            pubSubImplementation);

      continuousPlanningController = new QuadrupedContinuousPlanningController(defaultXGaitSettings, outputManager, robotDataReceiver, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2Topic controllerOutputTopic = ROS2Tools.getQuadrupedControllerOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedFootstepStatusMessage.class, controllerOutputTopic,
                                           s -> processFootstepStatusMessage(s.takeNextData()));

      // status messages from the planner
      ROS2Topic plannerOutputTopic = ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName)
                                                                   .withOutput();
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, PawStepPlanningToolboxOutputStatus.class, plannerOutputTopic,
                                           s -> processFootstepPlannerOutputMessage(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedXGaitSettingsPacket.class, getInputTopic(),
                                           s -> processQuadrupedXGaitSettings(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedContinuousPlanningRequestPacket.class, getInputTopic(),
                                           s -> processContinuousPlanningRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, PlanarRegionsListMessage.class, REACommunicationProperties.outputTopic,
                                           s -> processPlanarRegionsListMessage(s.takeNextData()));
   }

   @Override
   public Map<Class<? extends Settable<?>>, ROS2Topic> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, ROS2Topic> messages = new HashMap<>();

      messages.put(PawStepPlanningToolboxOutputStatus.class, getOutputTopic());
      messages.put(BodyPathPlanMessage.class, getOutputTopic());
      messages.put(QuadrupedTimedStepListMessage.class, getOutputTopic());

      ROS2Topic plannerInputTopic = ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName)
                                                                  .withInput();
      messages.put(PawStepPlanningRequestPacket.class, plannerInputTopic);
      messages.put(ToolboxStateMessage.class, plannerInputTopic);

      return messages;
   }

   private void processFootstepPlannerOutputMessage(PawStepPlanningToolboxOutputStatus footstepPlannerOutput)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processFootstepPlannerOutput(footstepPlannerOutput);
   }

   private void processContinuousPlanningRequest(QuadrupedContinuousPlanningRequestPacket planningRequestPacket)
   {
      if (continuousPlanningController != null)
      {
         continuousPlanningController.processContinuousPlanningRequest(planningRequestPacket);
         wakeUp();
      }
   }

   private void processFootstepStatusMessage(QuadrupedFootstepStatusMessage footstepStatusMessage)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processFootstepStatusMessage(footstepStatusMessage);
   }

   private void processPlanarRegionsListMessage(PlanarRegionsListMessage planarRegionsListMessage)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processPlanarRegionListMessage(planarRegionsListMessage);
   }

   private void processQuadrupedXGaitSettings(QuadrupedXGaitSettingsPacket quadrupedXGaitSettingsPacket)
   {
      if (continuousPlanningController != null)
         continuousPlanningController.processQuadrupedXGaitSettings(quadrupedXGaitSettingsPacket);
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
   public ROS2Topic getOutputTopic()
   {
      return ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic getInputTopic()
   {
      return ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX.withRobot(robotName).withInput();
   }

   @Override
   public void sleep()
   {
      super.sleep();

      ToolboxStateMessage plannerState = new ToolboxStateMessage();
      plannerState.setRequestedToolboxState(ToolboxStateMessage.SLEEP);

      outputManager.reportMessage(plannerState);
   }
}
