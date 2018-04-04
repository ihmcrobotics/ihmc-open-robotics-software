package us.ihmc.quadrupedRobotics.input.managers;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerEnum;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingStateEnum;
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

public class QuadrupedStepTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final PacketCommunicator packetCommunicator;
   private final QuadrupedXGaitStepStream stepStream;
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final QuadrupedPlanarVelocityInputProvider velocityInput = new QuadrupedPlanarVelocityInputProvider(null, registry);
   private final YoDouble timestamp = new YoDouble("timestamp", registry);
   private final YoBoolean walking = new YoBoolean("walking", registry);

   private final AtomicBoolean xGaitRequested = new AtomicBoolean();
   private final AtomicBoolean standingRequested = new AtomicBoolean();
   private final AtomicReference<QuadrupedControllerStateChangeMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicDouble desiredVelocityX = new AtomicDouble();
   private final AtomicDouble desiredVelocityY = new AtomicDouble();
   private final AtomicDouble desiredVelocityZ = new AtomicDouble();
   private final AtomicLong timestampNanos = new AtomicLong();

   public QuadrupedStepTeleopManager(PacketCommunicator packetCommunicator, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                     QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.packetCommunicator = packetCommunicator;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, null, registry);
      this.stepStream = new QuadrupedXGaitStepStream(velocityInput, xGaitSettings, referenceFrames, timestamp, registry);
      packetCommunicator.attachListener(QuadrupedControllerStateChangeMessage.class, controllerStateChangeMessage::set);
      packetCommunicator.attachListener(QuadrupedSteppingStateChangeMessage.class, steppingStateChangeMessage::set);
      packetCommunicator.attachListener(RobotConfigurationData.class, packet -> timestampNanos.set(packet.timestamp_));

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      velocityInput.set(desiredVelocityX.get(), desiredVelocityY.get(), desiredVelocityZ.get());

      if (xGaitRequested.getAndSet(false) && !isInStepState())
      {
         stepStream.onEntry();
         sendSteps();
         walking.set(true);
      }
      else if(standingRequested.getAndSet(false))
      {
         QuadrupedRequestedSteppingStateMessage stopWalkingPacket = new QuadrupedRequestedSteppingStateMessage();
         stopWalkingPacket.setQuadrupedSteppingState(QuadrupedSteppingRequestedEvent.REQUEST_STAND.toByte());
         packetCommunicator.send(stopWalkingPacket);
         walking.set(false);
      }
      else if (isInStepState() && walking.getBooleanValue())
      {
         stepStream.process();
         sendSteps();
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
      controllerMessage.setQuadrupedControllerName(QuadrupedForceControllerRequestedEvent.REQUEST_STEPPING.toByte());
      packetCommunicator.send(controllerMessage);
   }

   public void requestXGait()
   {
      xGaitRequested.set(true);
   }

   private boolean isInStepState()
   {
      QuadrupedControllerStateChangeMessage controllerStateChangeMessage = this.controllerStateChangeMessage.get();
      QuadrupedSteppingStateChangeMessage steppingStateChangeMessage = this.steppingStateChangeMessage.get();

      return (controllerStateChangeMessage != null && controllerStateChangeMessage.getEndControllerName() == QuadrupedForceControllerEnum.STEPPING.toByte()) &&
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

      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, true);
      packetCommunicator.send(message);
   }

   public YoQuadrupedXGaitSettings getXGaitSettings()
   {
      return xGaitSettings;
   }
}