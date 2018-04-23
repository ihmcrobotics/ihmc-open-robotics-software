package us.ihmc.quadrupedRobotics.input.managers;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettings;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final PacketCommunicator packetCommunicator;
   private final QuadrupedXGaitStepStream stepStream;
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoDouble timestamp = new YoDouble("timestamp", registry);
   private final YoBoolean walking = new YoBoolean("walking", registry);

   private final AtomicBoolean xGaitRequested = new AtomicBoolean();
   private final AtomicBoolean standingRequested = new AtomicBoolean();
   private final AtomicDouble desiredVelocityX = new AtomicDouble();
   private final AtomicDouble desiredVelocityY = new AtomicDouble();
   private final AtomicDouble desiredVelocityZ = new AtomicDouble();
   private final AtomicDouble desiredCoMHeight = new AtomicDouble();
   private final AtomicDouble desiredOrientationYaw = new AtomicDouble();
   private final AtomicDouble desiredOrientationPitch = new AtomicDouble();
   private final AtomicDouble desiredOrientationRoll = new AtomicDouble();
   private final AtomicDouble desiredOrientationTime = new AtomicDouble();

   private final AtomicReference<QuadrupedControllerStateChangeMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicLong timestampNanos = new AtomicLong();

   private final ComPositionPacket comPositionPacket = new ComPositionPacket();
   private final QuadrupedBodyOrientationMessage offsetBodyOrientationMessage = new QuadrupedBodyOrientationMessage();
   private final QuadrupedReferenceFrames referenceFrames;

   public QuadrupedTeleopManager(PacketCommunicator packetCommunicator, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                 double initialCoMHeight, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.packetCommunicator = packetCommunicator;
      this.referenceFrames = referenceFrames;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, null, registry);
      this.stepStream = new QuadrupedXGaitStepStream(xGaitSettings, referenceFrames, timestamp, registry);
      desiredCoMHeight.set(initialCoMHeight);

      packetCommunicator.attachListener(QuadrupedControllerStateChangeMessage.class, controllerStateChangeMessage::set);
      packetCommunicator.attachListener(QuadrupedSteppingStateChangeMessage.class, steppingStateChangeMessage::set);
      packetCommunicator.attachListener(RobotConfigurationData.class, packet -> timestampNanos.set(packet.timestamp_));

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      stepStream.setDesiredPlanarVelocity(desiredVelocityX.get(), desiredVelocityY.get(), desiredVelocityZ.get());
      referenceFrames.updateFrames();

      if (xGaitRequested.getAndSet(false) && !isInStepState())
      {
         stepStream.onEntry();
         sendSteps();
         walking.set(true);
      }
      else if(standingRequested.getAndSet(false))
      {
         requestStopWalking();
         walking.set(false);
      }
      else if (isInStepState() && walking.getBooleanValue())
      {
         stepStream.process();
         sendSteps();
      }
      else if (!walking.getBooleanValue())
      {
         sendBodyPose();
      }
   }

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      this.desiredVelocityX.set(desiredVelocityX);
      this.desiredVelocityY.set(desiredVelocityY);
      this.desiredVelocityZ.set(desiredVelocityZ);
   }

   public void requestSteppingState()
   {
      QuadrupedRequestedControllerStateMessage controllerMessage = new QuadrupedRequestedControllerStateMessage();
      controllerMessage.setQuadrupedControllerName(QuadrupedControllerRequestedEvent.REQUEST_STEPPING.toByte());
      packetCommunicator.send(controllerMessage);
   }

   public void requestStopWalking()
   {
      QuadrupedRequestedSteppingStateMessage steppingMessage = new QuadrupedRequestedSteppingStateMessage();
      steppingMessage.setQuadrupedSteppingState(QuadrupedSteppingRequestedEvent.REQUEST_STAND.toByte());
      packetCommunicator.send(steppingMessage);
   }

   public void requestXGait()
   {
      xGaitRequested.set(true);
   }

   private boolean isInStepState()
   {
      QuadrupedControllerStateChangeMessage controllerStateChangeMessage = this.controllerStateChangeMessage.get();
      QuadrupedSteppingStateChangeMessage steppingStateChangeMessage = this.steppingStateChangeMessage.get();

      return (controllerStateChangeMessage != null && controllerStateChangeMessage.getEndControllerName() == QuadrupedControllerEnum.STEPPING.toByte()) &&
            (steppingStateChangeMessage != null && steppingStateChangeMessage.getEndSteppingControllerName() == QuadrupedSteppingStateEnum.STEP.toByte());
   }

   public boolean isWalking()
   {
      return walking.getBooleanValue();
   }

   public void requestStanding()
   {
      standingRequested.set(true);
   }

   private void sendSteps()
   {
      List<? extends QuadrupedTimedStep> steps = stepStream.getSteps();
      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < steps.size(); i++)
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(steps.get(i)));

      QuadrupedTimedStepListMessage stepsMessage = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, true);
      packetCommunicator.send(stepsMessage);

      QuadrupedBodyOrientationMessage orientationMessage = QuadrupedMessageTools.createQuadrupedWorldFrameYawMessage(stepStream.getFootstepPlan().getPlannedSteps(), desiredVelocityZ.get());
      packetCommunicator.send(orientationMessage);
   }

   public void setDesiredCoMHeight(double desiredCoMHeight)
   {
      this.desiredCoMHeight.set(desiredCoMHeight);
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll, double time)
   {
      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
   }

   private void sendBodyPose()
   {
      double comHeight = desiredCoMHeight.getAndSet(Double.NaN);
      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

      if(!Double.isNaN(comHeight))
      {
         comPositionPacket.position.set(0.0, 0.0, comHeight);
         packetCommunicator.send(comPositionPacket);
      }

      if(!Double.isNaN(desiredYaw))
      {
         offsetBodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().clear();
         offsetBodyOrientationMessage.setIsAnOffsetOrientation(true);
         SO3TrajectoryPointMessage trajectoryPointMessage = offsetBodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().add();
         trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         trajectoryPointMessage.setTime(desiredTime);
         packetCommunicator.send(offsetBodyOrientationMessage);
      }
   }

   public YoQuadrupedXGaitSettings getXGaitSettings()
   {
      return xGaitSettings;
   }
}