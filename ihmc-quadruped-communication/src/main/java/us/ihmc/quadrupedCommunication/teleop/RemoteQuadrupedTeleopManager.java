package us.ihmc.quadrupedCommunication.teleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import javax.tools.Tool;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class RemoteQuadrupedTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrupedXGaitSettingsBasics xGaitSettings;
   private final Ros2Node ros2Node;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

   private final IHMCROS2Publisher<HighLevelStateMessage> controllerStatePublisher;
   private final IHMCROS2Publisher<QuadrupedRequestedSteppingStateMessage> steppingStatePublisher;
   private final IHMCROS2Publisher<QuadrupedTimedStepListMessage> timedStepListPublisher;
   private final IHMCROS2Publisher<QuadrupedBodyOrientationMessage> bodyOrientationPublisher;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionsListControllerPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingMessagePublisher;
   private final IHMCROS2Publisher<AbortWalkingMessage> abortWalkingMessagePublisher;
   private final IHMCROS2Publisher<QuadrupedFootLoadBearingMessage> loadBearingMessagePublisher;
   private final IHMCROS2Publisher<QuadrupedBodyHeightMessage> bodyHeightPublisher;

   private final IHMCROS2Publisher<QuadrupedTeleopDesiredVelocity> desiredVelocityPublisher;
   private final IHMCROS2Publisher<QuadrupedTeleopDesiredPose> desiredPosePublisher;

   private final IHMCROS2Publisher<ToolboxStateMessage> stepTeleopStatePublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> bodyTeleopStatePublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> footstepPlannerStatePublisher;

   private final IHMCROS2Publisher<QuadrupedXGaitSettingsPacket> stepXGaitSettingsPublisher;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionsListTeleopPublisher;
   private final IHMCROS2Publisher<QuadrupedBodyPathPlanMessage> bodyPathPublisher;

   private final IHMCROS2Publisher<QuadrupedXGaitSettingsPacket> plannerXGaitSettingsPublisher;
   private final IHMCROS2Publisher<QuadrupedFootstepPlanningRequestPacket> planningRequestPublisher;

   private final QuadrupedRobotDataReceiver robotDataReceiver;
   private final QuadrupedNetworkProcessor networkProcessor;
   private final String robotName;

   public RemoteQuadrupedTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedNetworkProcessor networkProcessor,
                                       FullQuadrupedRobotModel robotModel, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, YoVariableRegistry parentRegistry)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      this.networkProcessor = networkProcessor;

      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> controllerStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> steppingStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> robotConfigurationData.set(s.takeNextData()));

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      MessageTopicNameGenerator stepTeleopSubGenerator = getTopicNameGenerator(robotName, ROS2Tools.STEP_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator footstepPlannerSubGenerator = getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX,
                                                                                    ROS2Tools.ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator bodyTeleopSubGenerator = getTopicNameGenerator(robotName, ROS2Tools.BODY_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);

      controllerStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, controllerSubGenerator);
      steppingStatePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedRequestedSteppingStateMessage.class, controllerSubGenerator);
      timedStepListPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTimedStepListMessage.class, controllerSubGenerator);
      bodyOrientationPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyOrientationMessage.class, controllerSubGenerator);
      planarRegionsListControllerPublisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, controllerSubGenerator);
      pauseWalkingMessagePublisher = ROS2Tools.createPublisher(ros2Node, PauseWalkingMessage.class, controllerSubGenerator);
      abortWalkingMessagePublisher = ROS2Tools.createPublisher(ros2Node, AbortWalkingMessage.class, controllerSubGenerator);
      loadBearingMessagePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedFootLoadBearingMessage.class, controllerSubGenerator);
      bodyHeightPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyHeightMessage.class, controllerSubGenerator);

      desiredVelocityPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTeleopDesiredVelocity.class, stepTeleopSubGenerator);
      desiredPosePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTeleopDesiredPose.class, bodyTeleopSubGenerator);

      stepTeleopStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, stepTeleopSubGenerator);
      bodyTeleopStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, bodyTeleopSubGenerator);
      footstepPlannerStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, footstepPlannerSubGenerator);

      planarRegionsListTeleopPublisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, stepTeleopSubGenerator);
      stepXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, stepTeleopSubGenerator);
      bodyPathPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyPathPlanMessage.class, stepTeleopSubGenerator);

      plannerXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, footstepPlannerSubGenerator);
      planningRequestPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedFootstepPlanningRequestPacket.class, footstepPlannerSubGenerator);

      robotDataReceiver = new QuadrupedRobotDataReceiver(robotModel, null);

      parentRegistry.addChild(registry);

      initialize();
   }

   public String getRobotName()
   {
      return robotName;
   }

   public void publishTimedStepListToController(QuadrupedTimedStepListMessage message)
   {
      timedStepListPublisher.publish(message);
   }

   public void publishBodyOrientationMessage(QuadrupedBodyOrientationMessage message)
   {
      bodyOrientationPublisher.publish(message);
   }

   public void publishXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
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

   public void requestPauseWalking(boolean pauseWalking)
   {
      pauseWalkingMessagePublisher.publish(HumanoidMessageTools.createPauseWalkingMessage(pauseWalking));
   }

   public void requestAbortWalking()
   {
      abortWalkingMessagePublisher.publish(new AbortWalkingMessage());
   }

   public void requestLoadBearing(RobotQuadrant quadrant)
   {
      loadBearingMessagePublisher.publish(QuadrupedMessageTools.createLoadBearingMessage(quadrant));
   }

   private void requestStopWalking()
   {
      QuadrupedRequestedSteppingStateMessage steppingMessage = new QuadrupedRequestedSteppingStateMessage();
      steppingMessage.setQuadrupedSteppingRequestedEvent(QuadrupedSteppingRequestedEvent.REQUEST_STAND.toByte());
      steppingStatePublisher.publish(steppingMessage);
   }

   public void requestBodyTeleop()
   {
      bodyTeleopStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
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
      updateRobotModel();

      ReferenceFrame bodyHeightFrame = robotDataReceiver.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint3D bodyHeight = new FramePoint3D(bodyHeightFrame, 0.0, 0.0, desiredBodyHeight);
      bodyHeight.changeFrame(ReferenceFrame.getWorldFrame());

      QuadrupedBodyHeightMessage bodyHeightMessage = QuadrupedMessageTools.createQuadrupedBodyHeightMessage(0.0, bodyHeight.getZ());
      bodyHeightMessage.setControlBodyHeight(true);
      bodyHeightMessage.setIsExpressedInAbsoluteTime(false);

      bodyHeightPublisher.publish(bodyHeightMessage);
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
      planarRegionsListControllerPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      planarRegionsListTeleopPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
   }

   public void submitBodyPathPlan(QuadrupedBodyPathPlanMessage message)
   {
      bodyPathPublisher.publish(message);
   }

   public void setEndDoubleSupportDuration(QuadrupedSpeed speed, double endPhaseShift, double endDoubleSupportDuration)
   {
      if (endPhaseShift < 90.0)
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getPaceSlowTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         case MEDIUM:
            xGaitSettings.getPaceMediumTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         default:
            xGaitSettings.getPaceFastTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         }
      }
      else if (endPhaseShift < 180.0)
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getAmbleSlowTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         case MEDIUM:
            xGaitSettings.getAmbleMediumTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         default:
            xGaitSettings.getAmbleFastTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         }
      }
      else
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getTrotSlowTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         case MEDIUM:
            xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         default:
            xGaitSettings.getTrotFastTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         }
      }      publishXGaitSettings(xGaitSettings);
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      xGaitSettings.setEndPhaseShift(endPhaseShift);
      publishXGaitSettings(xGaitSettings);
   }

   public void setQuadrupedSpeed(QuadrupedSpeed speed)
   {
      xGaitSettings.setQuadrupedSpeed(speed);
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

   public void setStepDuration(QuadrupedSpeed speed, double endPhaseShift, double stepDuration)
   {
      if (endPhaseShift < 90.0)
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getPaceSlowTimings().setStepDuration(stepDuration);
            break;
         case MEDIUM:
            xGaitSettings.getPaceMediumTimings().setStepDuration(stepDuration);
            break;
         default:
            xGaitSettings.getPaceFastTimings().setStepDuration(stepDuration);
            break;
         }
      }
      else if (endPhaseShift < 180.0)
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getAmbleSlowTimings().setStepDuration(stepDuration);
            break;
         case MEDIUM:
            xGaitSettings.getAmbleMediumTimings().setStepDuration(stepDuration);
            break;
         default:
            xGaitSettings.getAmbleFastTimings().setStepDuration(stepDuration);
            break;
         }
      }
      else
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getTrotSlowTimings().setStepDuration(stepDuration);
            break;
         case MEDIUM:
            xGaitSettings.getTrotMediumTimings().setStepDuration(stepDuration);
            break;
         default:
            xGaitSettings.getTrotFastTimings().setStepDuration(stepDuration);
            break;
         }
      }
      publishXGaitSettings(xGaitSettings);
   }

   private void updateRobotModel()
   {
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if(robotConfigurationData != null)
      {
         robotDataReceiver.receivedPacket(robotConfigurationData);
         robotDataReceiver.updateRobotModel();
      }
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

   public QuadrupedXGaitSettingsBasics getXGaitSettings()
   {
      return xGaitSettings;
   }

   public Ros2Node getRos2Node()
   {
      return ros2Node;
   }
}