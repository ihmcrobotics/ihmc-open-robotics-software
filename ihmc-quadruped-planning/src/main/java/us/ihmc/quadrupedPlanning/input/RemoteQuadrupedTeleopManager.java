package us.ihmc.quadrupedPlanning.input;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedNetworkProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class RemoteQuadrupedTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoBoolean walking = new YoBoolean("walking", registry);
   private final Ros2Node ros2Node;

   private final AtomicBoolean paused = new AtomicBoolean(false);

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final IHMCROS2Publisher<HighLevelStateMessage> controllerStatePublisher;
   private final IHMCROS2Publisher<QuadrupedRequestedSteppingStateMessage> steppingStatePublisher;
   private final IHMCROS2Publisher<QuadrupedTimedStepListMessage> timedStepListPublisher;

   private final IHMCROS2Publisher<QuadrupedTeleopDesiredVelocity> desiredVelocityPublisher;
   private final IHMCROS2Publisher<QuadrupedTeleopDesiredHeight> desiredHeightPublisher;
   private final IHMCROS2Publisher<QuadrupedTeleopDesiredPose> desiredPosePublisher;

   private final IHMCROS2Publisher<ToolboxStateMessage> stepTeleopStatePublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> bodyTeleopStatePublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> heightTeleopStatePublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> footstepPlannerStatePublisher;

   private final IHMCROS2Publisher<QuadrupedXGaitSettingsPacket> stepXGaitSettingsPublisher;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionsListPublisher;
   private final IHMCROS2Publisher<QuadrupedBodyPathPlanMessage> bodyPathPublisher;

   private final IHMCROS2Publisher<QuadrupedXGaitSettingsPacket> plannerXGaitSettingsPublisher;
   private final IHMCROS2Publisher<QuadrupedFootstepPlanningRequestPacket> planningRequestPublisher;

   private final QuadrupedNetworkProcessor networkProcessor;

   public RemoteQuadrupedTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedNetworkProcessor networkProcessor,
                                       QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, YoVariableRegistry parentRegistry)
   {
      this.ros2Node = ros2Node;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      this.networkProcessor = networkProcessor;

      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> controllerStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> steppingStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> paused.set(true));

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      MessageTopicNameGenerator stepTeleopSubGenerator = getTopicNameGenerator(robotName, ROS2Tools.STEP_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator footstepPlannerSubGenerator = getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX,
                                                                                    ROS2Tools.ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator heightTeleopSubGenerator = getTopicNameGenerator(robotName, ROS2Tools.HEIGHT_TELEOP_TOOLBOX,
                                                                                 ROS2Tools.ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator bodyTeleopSubGenerator = getTopicNameGenerator(robotName, ROS2Tools.BODY_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);

      controllerStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, controllerSubGenerator);
      steppingStatePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedRequestedSteppingStateMessage.class, controllerSubGenerator);
      timedStepListPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTimedStepListMessage.class, controllerSubGenerator);

      desiredVelocityPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTeleopDesiredVelocity.class, stepTeleopSubGenerator);
      desiredPosePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTeleopDesiredPose.class, bodyTeleopSubGenerator);
      desiredHeightPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTeleopDesiredHeight.class, heightTeleopSubGenerator);


      stepTeleopStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, stepTeleopSubGenerator);
      heightTeleopStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, heightTeleopSubGenerator);
      bodyTeleopStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, bodyTeleopSubGenerator);
      footstepPlannerStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, footstepPlannerSubGenerator);

      planarRegionsListPublisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, stepTeleopSubGenerator);
      stepXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, stepTeleopSubGenerator);
      bodyPathPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyPathPlanMessage.class, stepTeleopSubGenerator);

      plannerXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, footstepPlannerSubGenerator);
      planningRequestPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedFootstepPlanningRequestPacket.class, footstepPlannerSubGenerator);

      parentRegistry.addChild(registry);

      initialize();
   }

   public void publishTimedStepListToController(QuadrupedTimedStepListMessage message)
   {
      timedStepListPublisher.publish(message);
   }

   public void publishXGaitSettings(YoQuadrupedXGaitSettings xGaitSettings)
   {
      stepXGaitSettingsPublisher.publish(xGaitSettings.getAsPacket());
      plannerXGaitSettingsPublisher.publish(xGaitSettings.getAsPacket());
   }

   public void publishPlanningRequest(QuadrupedFootstepPlanningRequestPacket packet)
   {
      footstepPlannerStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      planningRequestPublisher.publish(packet);
   }

   public void initialize()
   {
      heightTeleopStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      bodyTeleopStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
   }

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      desiredVelocityPublisher.publish(QuadrupedMessageTools.createQuadrupedTeleopDesiredVelocity(desiredVelocityX, desiredVelocityY, desiredVelocityZ));
   }

   public void requestStandPrep()
   {
      controllerStatePublisher.publish(HumanoidMessageTools.createHighLevelStateMessage(HighLevelControllerName.STAND_PREP_STATE));
   }

   public void requestWalkingState()
   {
      controllerStatePublisher.publish(HumanoidMessageTools.createHighLevelStateMessage(HighLevelControllerName.STAND_TRANSITION_STATE));
   }

   private void requestStopWalking()
   {
      QuadrupedRequestedSteppingStateMessage steppingMessage = new QuadrupedRequestedSteppingStateMessage();
      steppingMessage.setQuadrupedSteppingRequestedEvent(QuadrupedSteppingRequestedEvent.REQUEST_STAND.toByte());
      steppingStatePublisher.publish(steppingMessage);
   }

   public void requestXGait()
   {
      stepTeleopStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
   }

   public void requestStanding()
   {
      requestStopWalking();
      stepTeleopStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      desiredHeightPublisher.publish(QuadrupedMessageTools.createQuadrupedTeleopDesiredHeight(desiredBodyHeight));
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll, double time)
   {
      desiredPosePublisher.publish(QuadrupedMessageTools.createQuadrupedTeleopDesiredPose(yaw, pitch, roll, time));
   }

   public void setDesiredBodyTranslation(double x, double y, double time)
   {
      desiredPosePublisher.publish(QuadrupedMessageTools.createQuadrupedTeleopDesiredPose(x, y, time));
   }


   public void setDesiredBodyPose(double x, double y, double yaw, double pitch, double roll, double time)
   {
      desiredPosePublisher.publish(QuadrupedMessageTools.createQuadrupedTeleopDesiredPose(x, y, yaw, pitch, roll, time));
   }

   public void submitPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      planarRegionsListPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
   }

   public void submitBodyPathPlan(QuadrupedBodyPathPlanMessage message)
   {
      bodyPathPublisher.publish(message);
   }

   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      xGaitSettings.setEndDoubleSupportDuration(endDoubleSupportDuration);
      publishXGaitSettings(xGaitSettings);
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      xGaitSettings.setEndPhaseShift(endPhaseShift);
      publishXGaitSettings(xGaitSettings);
   }

   public void setStanceWidth(double stanceWidth)
   {
      xGaitSettings.setStanceWidth(stanceWidth);
      publishXGaitSettings(xGaitSettings);
   }

   public void setStanceLength(double stanceLength)
   {
      xGaitSettings.setStanceLength(stanceLength);
      publishXGaitSettings(xGaitSettings);
   }

   public void setStepGroundClearance(double groundClearance)
   {
      xGaitSettings.setStepGroundClearance(groundClearance);
      publishXGaitSettings(xGaitSettings);
   }

   public void setStepDuration(double stepDuration)
   {
      xGaitSettings.setStepDuration(stepDuration);
      publishXGaitSettings(xGaitSettings);
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shift)
   {
      networkProcessor.setShiftPlanBasedOnStepAdjustment(shift);
   }

   public void setXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.xGaitSettings.set(xGaitSettings);
      publishXGaitSettings(this.xGaitSettings);
   }

   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      return xGaitSettings;
   }

   public void setPaused(boolean pause)
   {
      paused.set(pause);

      steppingStateChangeMessage.set(null);
      walking.set(false);
   }

   public boolean isPaused()
   {
      return paused.get();
   }

   public Ros2Node getRos2Node()
   {
      return ros2Node;
   }
}