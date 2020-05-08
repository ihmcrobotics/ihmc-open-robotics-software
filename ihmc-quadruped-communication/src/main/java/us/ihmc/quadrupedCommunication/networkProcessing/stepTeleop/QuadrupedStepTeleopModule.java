package us.ihmc.quadrupedCommunication.networkProcessing.stepTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2TopicName;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class QuadrupedStepTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 10;

   private final QuadrupedStepTeleopController stepTeleopController;

   public QuadrupedStepTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters, LogModelProvider modelProvider, boolean startYoVariableServer,
                                    boolean logYoVariables, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, startYoVariableServer,
            new DataServerSettings(logYoVariables, "StepTeleopModule"), updatePeriodMilliseconds, pubSubImplementation);


      stepTeleopController = new QuadrupedStepTeleopController(defaultXGaitSettings, pointFootSnapperParameters, outputManager, robotDataReceiver, registry,
                                                               yoGraphicsListRegistry, updatePeriodMilliseconds);
      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2TopicName controllerOutputTopicName = ROS2Tools.getQuadrupedControllerOutputTopicName(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, RobotConfigurationData.class, controllerOutputTopicName,
                                           s -> processTimestamp(s.takeNextData().getMonotonicTime()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, HighLevelStateMessage.class, controllerOutputTopicName, s -> setPaused(true));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerOutputTopicName,
                                           s -> processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedFootstepStatusMessage.class, controllerOutputTopicName,
                                           s -> processFootstepStatusMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerOutputTopicName,
                                           s -> processSteppingStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedGroundPlaneMessage.class, controllerOutputTopicName,
                                           s -> processGroundPlaneMessage(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedBodyPathPlanMessage.class, getSubscriberTopicName(),
                                           s -> processBodyPathPlanMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedXGaitSettingsPacket.class, getSubscriberTopicName(),
                                           s -> processXGaitSettingsPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedTeleopDesiredVelocity.class, getSubscriberTopicName(),
                                           s -> processTeleopDesiredVelocity(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, PlanarRegionsListMessage.class, getSubscriberTopicName(),
                                           s -> processPlanarRegionsListMessage(s.takeNextData()));
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

   private void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
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
   public Map<Class<? extends Settable<?>>, ROS2TopicName> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, ROS2TopicName> messages = new HashMap<>();

      ROS2TopicName controllerInputTopicName = ROS2Tools.getQuadrupedControllerInputTopicName(robotName);
      messages.put(QuadrupedTimedStepListMessage.class, controllerInputTopicName);
      messages.put(QuadrupedBodyOrientationMessage.class, controllerInputTopicName);
      messages.put(AbortWalkingMessage.class, controllerInputTopicName);

      return messages;
   }

   @Override
   public ROS2TopicName getPublisherTopicName()
   {
      return ROS2Tools.STEP_TELEOP_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2TopicName getSubscriberTopicName()
   {
      return ROS2Tools.STEP_TELEOP_TOOLBOX.withRobot(robotName).withInput();
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
