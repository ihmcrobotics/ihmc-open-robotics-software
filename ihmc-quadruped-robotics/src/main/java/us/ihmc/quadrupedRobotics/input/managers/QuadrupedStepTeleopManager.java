package us.ihmc.quadrupedRobotics.input.managers;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.packets.*;
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
   private final AtomicReference<QuadrupedForceControllerStatePacket> forceControlStatePacket = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStatePacket> steppingStatePacket = new AtomicReference<>();
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
      packetCommunicator.attachListener(QuadrupedForceControllerStatePacket.class, forceControlStatePacket::set);
      packetCommunicator.attachListener(QuadrupedSteppingStatePacket.class, steppingStatePacket::set);
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
         QuadrupedSteppingEventPacket stopWalkingPacket = new QuadrupedSteppingEventPacket(QuadrupedSteppingRequestedEvent.REQUEST_STAND);
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
      QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_STEPPING);
      packetCommunicator.send(eventPacket);
   }

   public void requestXGait()
   {
      xGaitRequested.set(true);
   }

   private boolean isInStepState()
   {
      QuadrupedForceControllerStatePacket forceControllerStatePacket = this.forceControlStatePacket.get();
      QuadrupedSteppingStatePacket steppingStatePacket = this.steppingStatePacket.get();
      return (forceControllerStatePacket != null && forceControllerStatePacket.get() == QuadrupedForceControllerEnum.STEPPING) && (steppingStatePacket != null
            && steppingStatePacket.get() == QuadrupedSteppingStateEnum.STEP);
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
      QuadrupedTimedStepPacket timedStepPacket = new QuadrupedTimedStepPacket(steps, true);
      packetCommunicator.send(timedStepPacket);
   }

   public YoQuadrupedXGaitSettings getXGaitSettings()
   {
      return xGaitSettings;
   }
}