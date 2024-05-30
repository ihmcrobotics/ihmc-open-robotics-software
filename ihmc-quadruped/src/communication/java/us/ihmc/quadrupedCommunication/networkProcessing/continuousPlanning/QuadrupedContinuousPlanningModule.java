package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import static us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor.continuousPlanningPort;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import toolbox_msgs.msg.dds.BodyPathPlanMessage;
import quadruped_msgs.msg.dds.PawStepPlanningRequestPacket;
import quadruped_msgs.msg.dds.PawStepPlanningToolboxOutputStatus;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import quadruped_msgs.msg.dds.QuadrupedContinuousPlanningRequestPacket;
import quadruped_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import quadruped_msgs.msg.dds.QuadrupedTimedStepListMessage;
import quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.communication.FootstepPlannerAPI;
import us.ihmc.communication.QuadrupedAPI;
import us.ihmc.communication.ToolboxAPIs;
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
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

public class QuadrupedContinuousPlanningModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 50;

   private final QuadrupedContinuousPlanningController continuousPlanningController;

   public QuadrupedContinuousPlanningModule(FullQuadrupedRobotModelFactory modelFactory,
                                            QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                            LogModelProvider modelProvider,
                                            boolean startYoVariableServer,
                                            boolean logYoVariables,
                                            DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(modelFactory.getRobotDefinition().getName(), modelFactory.createFullRobotModel(), defaultXGaitSettings, modelProvider, startYoVariableServer,
           logYoVariables, pubSubImplementation);
   }

   public QuadrupedContinuousPlanningModule(String name,
                                            FullQuadrupedRobotModel fulRobotModel,
                                            QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                            LogModelProvider modelProvider,
                                            boolean startYoVariableServer,
                                            boolean logYoVariables,
                                            DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(name, fulRobotModel, modelProvider, startYoVariableServer,
            new DataServerSettings(logYoVariables, true, continuousPlanningPort, "ContinuousPlanningModule"), updatePeriodMilliseconds, pubSubImplementation);

      continuousPlanningController = new QuadrupedContinuousPlanningController(defaultXGaitSettings, outputManager, robotDataReceiver, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeROS2Node realtimeROS2Node)
   {
      // status messages from the controller
      ROS2Topic<?> controllerOutputTopic = QuadrupedAPI.getQuadrupedControllerOutputTopic(robotName);
      realtimeROS2Node.createSubscription(controllerOutputTopic.withTypeName(QuadrupedFootstepStatusMessage.class),
                                          s -> processFootstepStatusMessage(s.takeNextData()));

      // status messages from the planner
      ROS2Topic<?> plannerOutputTopic = FootstepPlannerAPI.FOOTSTEP_PLANNER.withRobot(robotName).withOutput();
      realtimeROS2Node.createSubscription(plannerOutputTopic.withTypeName(PawStepPlanningToolboxOutputStatus.class),
                                          s -> processFootstepPlannerOutputMessage(s.takeNextData()));

      // inputs to this module
      realtimeROS2Node.createSubscription(getInputTopic().withTypeName(QuadrupedXGaitSettingsPacket.class),
                                          s -> processQuadrupedXGaitSettings(s.takeNextData()));
      realtimeROS2Node.createSubscription(getInputTopic().withTypeName(QuadrupedContinuousPlanningRequestPacket.class),
                                          s -> processContinuousPlanningRequest(s.takeNextData()));
      realtimeROS2Node.createSubscription(REACommunicationProperties.outputTopic.withTypeName(PlanarRegionsListMessage.class),
                                          s -> processPlanarRegionsListMessage(s.takeNextData()));
   }

   @Override
   public Map<Class<? extends Settable<?>>, ROS2Topic> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, ROS2Topic> messages = new HashMap<>();

      messages.put(PawStepPlanningToolboxOutputStatus.class, getOutputTopic());
      messages.put(BodyPathPlanMessage.class, getOutputTopic());
      messages.put(QuadrupedTimedStepListMessage.class, getOutputTopic());

      ROS2Topic plannerInputTopic = FootstepPlannerAPI.FOOTSTEP_PLANNER.withRobot(robotName).withInput();
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
   public ROS2Topic<?> getOutputTopic()
   {
      return ToolboxAPIs.CONTINUOUS_PLANNING_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return ToolboxAPIs.CONTINUOUS_PLANNING_TOOLBOX.withRobot(robotName).withInput();
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
