package us.ihmc.quadrupedCommunication.networkProcessing.stepTeleop;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.AbortWalkingMessage;
import ihmc_common_msgs.msg.dds.GroundPlaneMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage;
import quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import quadruped_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import quadruped_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import quadruped_msgs.msg.dds.QuadrupedTeleopDesiredVelocity;
import quadruped_msgs.msg.dds.QuadrupedTimedStepListMessage;
import quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.QuadrupedAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

public class QuadrupedStepTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 10;

   private final QuadrupedStepTeleopController stepTeleopController;

   public QuadrupedStepTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters, LogModelProvider modelProvider, boolean startYoVariableServer,
                                    boolean logYoVariables, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(modelFactory.getRobotDefinition().getName(), modelFactory.createFullRobotModel(), modelProvider, startYoVariableServer,
            new DataServerSettings(logYoVariables, "StepTeleopModule"), updatePeriodMilliseconds, pubSubImplementation);


      stepTeleopController = new QuadrupedStepTeleopController(defaultXGaitSettings, pointFootSnapperParameters, outputManager, robotDataReceiver, registry,
                                                               yoGraphicsListRegistry, updatePeriodMilliseconds);
      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeROS2Node realtimeROS2Node)
   {
      // status messages from the controller
      ROS2Topic<?> controllerOutputTopic = QuadrupedAPI.getQuadrupedControllerOutputTopic(robotName);
      realtimeROS2Node.createSubscription(controllerOutputTopic.withTypeName(RobotConfigurationData.class),
                                          s -> processTimestamp(s.takeNextData().getMonotonicTime()));
      realtimeROS2Node.createSubscription(controllerOutputTopic.withTypeName(HighLevelStateMessage.class), s -> setPaused(true));
      realtimeROS2Node.createSubscription(controllerOutputTopic.withTypeName(HighLevelStateChangeStatusMessage.class),
                                          s -> processHighLevelStateChangeMessage(s.takeNextData()));
      realtimeROS2Node.createSubscription(controllerOutputTopic.withTypeName(QuadrupedFootstepStatusMessage.class),
                                          s -> processFootstepStatusMessage(s.takeNextData()));
      realtimeROS2Node.createSubscription(controllerOutputTopic.withTypeName(QuadrupedSteppingStateChangeMessage.class),
                                          s -> processSteppingStateChangeMessage(s.takeNextData()));
      realtimeROS2Node.createSubscription(controllerOutputTopic.withTypeName(GroundPlaneMessage.class), s -> processGroundPlaneMessage(s.takeNextData()));

      // inputs to this module
      realtimeROS2Node.createSubscription(getInputTopic().withTypeName(QuadrupedBodyPathPlanMessage.class), s -> processBodyPathPlanMessage(s.takeNextData()));
      realtimeROS2Node.createSubscription(getInputTopic().withTypeName(QuadrupedXGaitSettingsPacket.class), s -> processXGaitSettingsPacket(s.takeNextData()));
      realtimeROS2Node.createSubscription(getInputTopic().withTypeName(QuadrupedTeleopDesiredVelocity.class), s -> processTeleopDesiredVelocity(s.takeNextData()));
      realtimeROS2Node.createSubscription(getInputTopic().withTypeName(PlanarRegionsListMessage.class), s -> processPlanarRegionsListMessage(s.takeNextData()));
   }

   private void processTimestamp(long timestamp)
   {
      if (stepTeleopController != null)
         stepTeleopController.processTimestamp(timestamp);
   }

   private void setPaused(boolean paused)
   {
      if (stepTeleopController != null)
         stepTeleopController.setPaused(paused);
   }

   private void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      if (stepTeleopController != null)
         stepTeleopController.processHighLevelStateChangeMessage(message);
   }

   private void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      if (stepTeleopController != null)
         stepTeleopController.processFootstepStatusMessage(message);
   }

   private void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      if (stepTeleopController != null)
         stepTeleopController.processSteppingStateChangeMessage(message);
   }

   private void processGroundPlaneMessage(GroundPlaneMessage message)
   {
      if (stepTeleopController != null)
         stepTeleopController.processGroundPlaneMessage(message);
   }

   private void processBodyPathPlanMessage(QuadrupedBodyPathPlanMessage message)
   {
      if (stepTeleopController != null)
         stepTeleopController.processBodyPathPlanMessage(message);
   }

   private void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      if (stepTeleopController != null)
         stepTeleopController.processXGaitSettingsPacket(packet);
   }

   private void processTeleopDesiredVelocity(QuadrupedTeleopDesiredVelocity message)
   {
      if (stepTeleopController != null)
         stepTeleopController.processTeleopDesiredVelocity(message);
   }

   private void processPlanarRegionsListMessage(PlanarRegionsListMessage message)
   {
      if (stepTeleopController != null)
         stepTeleopController.processPlanarRegionsListMessage(message);
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return stepTeleopController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public Map<Class<? extends Settable<?>>, ROS2Topic> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, ROS2Topic> messages = new HashMap<>();

      ROS2Topic controllerInputTopic = QuadrupedAPI.getQuadrupedControllerInputTopic(robotName);
      messages.put(QuadrupedTimedStepListMessage.class, controllerInputTopic);
      messages.put(QuadrupedBodyOrientationMessage.class, controllerInputTopic);
      messages.put(AbortWalkingMessage.class, controllerInputTopic);

      return messages;
   }

   @Override
   public ROS2Topic<?> getOutputTopic()
   {
      return ToolboxAPIs.STEP_TELEOP_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return ToolboxAPIs.STEP_TELEOP_TOOLBOX.withRobot(robotName).withInput();
   }

   @Override
   public void sleep()
   {
      stepTeleopController.setPaused(true);
      super.sleep();
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shift)
   {
      stepTeleopController.setShiftPlanBasedOnStepAdjustment(shift);
   }


}
