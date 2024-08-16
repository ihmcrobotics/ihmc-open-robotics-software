package us.ihmc.quadrupedCommunication.teleop;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import quadruped_msgs.msg.dds.*;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.communication.FootstepPlannerAPI;
import us.ihmc.communication.QuadrupedAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
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
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class RemoteQuadrupedTeleopManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final QuadrupedXGaitSettingsBasics xGaitSettings;
   private final ROS2Node ros2Node;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

   private final ROS2PublisherBasics<HighLevelStateMessage> controllerStatePublisher;
   private final ROS2PublisherBasics<QuadrupedRequestedSteppingStateMessage> steppingStatePublisher;
   private final ROS2PublisherBasics<QuadrupedTimedStepListMessage> timedStepListPublisher;
   private final ROS2PublisherBasics<QuadrupedBodyOrientationMessage> bodyOrientationPublisher;
   private final ROS2PublisherBasics<PlanarRegionsListMessage> planarRegionsListControllerPublisher;
   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingMessagePublisher;
   private final ROS2PublisherBasics<AbortWalkingMessage> abortWalkingMessagePublisher;
   private final ROS2PublisherBasics<QuadrupedFootLoadBearingMessage> loadBearingMessagePublisher;
   private final ROS2PublisherBasics<QuadrupedBodyHeightMessage> bodyHeightPublisher;
   private final ROS2PublisherBasics<QuadrupedBodyTrajectoryMessage> bodyPosePublisher;

   private final ROS2PublisherBasics<QuadrupedTeleopDesiredVelocity> desiredVelocityPublisher;

   private final ROS2PublisherBasics<ToolboxStateMessage> stepTeleopStatePublisher;
   private final ROS2PublisherBasics<ToolboxStateMessage> pawPlannerStatePublisher;

   private final ROS2PublisherBasics<QuadrupedXGaitSettingsPacket> stepXGaitSettingsPublisher;
   private final ROS2PublisherBasics<PlanarRegionsListMessage> planarRegionsListTeleopPublisher;
   private final ROS2PublisherBasics<QuadrupedBodyPathPlanMessage> bodyPathPublisher;

   private final ROS2PublisherBasics<QuadrupedXGaitSettingsPacket> plannerXGaitSettingsPublisher;
   private final ROS2PublisherBasics<PawStepPlanningRequestPacket> planningRequestPublisher;

   private final AtomicDouble timestamp = new AtomicDouble();
   private final QuadrupedRobotDataReceiver robotDataReceiver;
   private final QuadrupedNetworkProcessor networkProcessor;
   private final String robotName;

   public RemoteQuadrupedTeleopManager(String robotName, ROS2Node ros2Node, QuadrupedNetworkProcessor networkProcessor,
                                       FullQuadrupedRobotModel robotModel, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, YoRegistry parentRegistry)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      this.networkProcessor = networkProcessor;

      ROS2Topic<?> controllerOutputTopic = QuadrupedAPI.getQuadrupedControllerOutputTopic(robotName);
      ros2Node.createSubscription(controllerOutputTopic.withTypeName(HighLevelStateChangeStatusMessage.class),
                                  s -> controllerStateChangeMessage.set(s.takeNextData()));
      ros2Node.createSubscription(controllerOutputTopic.withTypeName(QuadrupedSteppingStateChangeMessage.class),
                                  s -> steppingStateChangeMessage.set(s.takeNextData()));
      ros2Node.createSubscription(controllerOutputTopic.withTypeName(RobotConfigurationData.class),
                                  s -> robotConfigurationData.set(s.takeNextData()));

      ROS2Topic controllerInputTopic = QuadrupedAPI.getQuadrupedControllerInputTopic(robotName);
      ROS2Topic stepTeleopInputTopic = ToolboxAPIs.STEP_TELEOP_TOOLBOX.withRobot(robotName)
                                                                      .withInput();
      ROS2Topic footstepPlannerInputTopic = FootstepPlannerAPI.FOOTSTEP_PLANNER.withRobot(robotName)
                                                                               .withInput();

      controllerStatePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(HighLevelStateMessage.class));
      steppingStatePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedRequestedSteppingStateMessage.class));
      timedStepListPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedTimedStepListMessage.class));
      bodyOrientationPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedBodyOrientationMessage.class));
      planarRegionsListControllerPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(PlanarRegionsListMessage.class));
      pauseWalkingMessagePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(PauseWalkingMessage.class));
      abortWalkingMessagePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(AbortWalkingMessage.class));
      loadBearingMessagePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedFootLoadBearingMessage.class));
      bodyHeightPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedBodyHeightMessage.class));
      bodyPosePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedBodyTrajectoryMessage.class));

      desiredVelocityPublisher = ros2Node.createPublisher(stepTeleopInputTopic.withTypeName(QuadrupedTeleopDesiredVelocity.class));

      stepTeleopStatePublisher = ros2Node.createPublisher(stepTeleopInputTopic.withTypeName(ToolboxStateMessage.class));
      pawPlannerStatePublisher = ros2Node.createPublisher(footstepPlannerInputTopic.withTypeName(ToolboxStateMessage.class));

      planarRegionsListTeleopPublisher = ros2Node.createPublisher(stepTeleopInputTopic.withTypeName(PlanarRegionsListMessage.class));
      stepXGaitSettingsPublisher = ros2Node.createPublisher(stepTeleopInputTopic.withTypeName(QuadrupedXGaitSettingsPacket.class));
      bodyPathPublisher = ros2Node.createPublisher(stepTeleopInputTopic.withTypeName(QuadrupedBodyPathPlanMessage.class));

      plannerXGaitSettingsPublisher = ros2Node.createPublisher(footstepPlannerInputTopic.withTypeName(QuadrupedXGaitSettingsPacket.class));
      planningRequestPublisher = ros2Node.createPublisher(footstepPlannerInputTopic.withTypeName(PawStepPlanningRequestPacket.class));

      robotDataReceiver = new QuadrupedRobotDataReceiver(robotModel, null);

      parentRegistry.addChild(registry);
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

   public void publishPlanningRequest(PawStepPlanningRequestPacket packet)
   {
      pawPlannerStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      planningRequestPublisher.publish(packet);
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
      abortWalkingMessagePublisher.publish(new AbortWalkingMessage());
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

   public void setDesiredBodyOrientation(double desiredYaw, double desiredPitch, double desiredRoll, double time)
   {
      updateRobotModel();

      QuadrupedBodyTrajectoryMessage bodyTrajectoryMessage = new QuadrupedBodyTrajectoryMessage();
      SE3TrajectoryMessage se3Trajectory = bodyTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      se3Trajectory.getAngularSelectionMatrix().setXSelected(true);
      se3Trajectory.getAngularSelectionMatrix().setYSelected(true);
      se3Trajectory.getAngularSelectionMatrix().setZSelected(true);
      se3Trajectory.getLinearSelectionMatrix().setXSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setYSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setZSelected(false);
      SE3TrajectoryPointMessage trajectoryPointMessage = se3Trajectory.getTaskspaceTrajectoryPoints().add();
      trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
      trajectoryPointMessage.setTime(time + timestamp.get());

      bodyPosePublisher.publish(bodyTrajectoryMessage);
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
         timestamp.set(1e-9 * robotConfigurationData.getMonotonicTime());
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

   public ROS2Node getROS2Node()
   {
      return ros2Node;
   }
}