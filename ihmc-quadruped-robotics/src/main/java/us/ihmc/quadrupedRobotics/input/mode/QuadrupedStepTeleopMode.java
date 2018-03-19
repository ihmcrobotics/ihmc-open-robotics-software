package us.ihmc.quadrupedRobotics.input.mode;

import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.RobotConfigurationData;
import net.java.games.input.Event;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerStatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedSteppingStatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerEnum;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettings;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedStepTeleopMode implements QuadrupedTeleopMode
{
   private static final double DT = 0.01;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedXGaitStepStream stepStream;
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final QuadrupedPlanarVelocityInputProvider velocityInput = new QuadrupedPlanarVelocityInputProvider(null, registry);
   private final YoDouble timestamp = new YoDouble("timestamp", registry);

   private final DoubleParameter yawScaleParameter = new DoubleParameter("yawScale", registry, 0.15);
   private final DoubleParameter pitchScaleParameter = new DoubleParameter("pitchScale", registry, 0.15);
   private final DoubleParameter zdotScaleParameter = new DoubleParameter("zdotScale", registry, 0.25);
   private final DoubleParameter xStrideMax = new DoubleParameter("xStrideMax", registry, 0.4);
   private final DoubleParameter yStrideMax = new DoubleParameter("yStrideMax", registry, 0.25);
   private final DoubleParameter yawRateScale = new DoubleParameter("yawRateScale", registry, 0.25);

   // xgait step parameters
   private final DoubleParameter[] xGaitStepDuration = new DoubleParameter[2];
   private final DoubleParameter[] xGaitEndDoubleSupportDuration = new DoubleParameter[2];
   private final DoubleParameter[] xGaitEndPhaseShift = new DoubleParameter[2];

   private final PacketCommunicator packetCommunicator;
   private InputValueIntegrator comZ;

   private final AtomicBoolean xGaitRequested = new AtomicBoolean();
   private final AtomicReference<QuadrupedForceControllerStatePacket> forceControlStatePacket = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStatePacket> steppingStatePacket = new AtomicReference<>();
   private final AtomicLong timestampNanos = new AtomicLong();

   public QuadrupedStepTeleopMode(PacketCommunicator packetCommunicator, QuadrupedPhysicalProperties physicalProperties, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                  QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.packetCommunicator = packetCommunicator;

      this.comZ = new InputValueIntegrator(DT, physicalProperties.getNominalCoMHeight());
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, null, registry);
      this.stepStream = new QuadrupedXGaitStepStream(velocityInput, xGaitSettings, referenceFrames, timestamp, registry);

      xGaitStepDuration[0] = new DoubleParameter("xGaitStepDurationMode0", registry, 0.5);
      xGaitStepDuration[1] = new DoubleParameter("xGaitStepDurationMode1", registry, 0.33);
      xGaitEndDoubleSupportDuration[0] = new DoubleParameter("xGaitEndDoubleSupportDurationMode0", registry, 1.0);
      xGaitEndDoubleSupportDuration[1] = new DoubleParameter("xGaitEndDoubleSupportDurationMode1", registry, 0.05);
      xGaitEndPhaseShift[0] = new DoubleParameter("xGaitEndPhaseShiftMode0", registry, 90);
      xGaitEndPhaseShift[1] = new DoubleParameter("xGaitEndPhaseShiftMode1", registry, 180);

      packetCommunicator.attachListener(QuadrupedForceControllerStatePacket.class, forceControlStatePacket::set);
      packetCommunicator.attachListener(QuadrupedSteppingStatePacket.class, steppingStatePacket::set);
      packetCommunicator.attachListener(RobotConfigurationData.class, packet -> timestampNanos.set(packet.getTimestamp()));

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void update(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimates estimates)
   {
      double bodyRoll = 0.0;
      double bodyPitch = channels.get(XBoxOneMapping.RIGHT_STICK_Y) * pitchScaleParameter.getValue();
      double bodyYaw = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawScaleParameter.getValue();
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(bodyYaw, bodyPitch, bodyRoll);
      packetCommunicator.send(orientationPacket);

      double comZdot = 0.0;
      if (channels.get(XBoxOneMapping.DPAD) == 0.25)
      {
         comZdot += zdotScaleParameter.getValue();
      }
      if (channels.get(XBoxOneMapping.DPAD) == 0.75)
      {
         comZdot -= zdotScaleParameter.getValue();
      }
      ComPositionPacket comPositionPacket = new ComPositionPacket(0.0, 0.0, comZ.update(comZdot));
      packetCommunicator.send(comPositionPacket);

      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      double xVelocityMax = 0.5 * xStrideMax.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double yVelocityMax = 0.5 * yStrideMax.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double yawRateMax = yawRateScale.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double xVelocity = channels.get(XBoxOneMapping.LEFT_STICK_Y) * xVelocityMax;
      double yVelocity = channels.get(XBoxOneMapping.LEFT_STICK_X) * yVelocityMax;
      double yawRate = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawRateMax;
      velocityInput.set(xVelocity, yVelocity, yawRate);

      if(xGaitRequested.getAndSet(false) && !isInStepState())
      {
         stepStream.onEntry();
         sendSteps();
      }

      if(isInStepState())
      {
         stepStream.process();
         sendSteps();
      }
   }

   @Override
   public void onInputEvent(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimates estimates, Event event)
   {
      if (event.getValue() < 0.5)
         return;

      if(XBoxOneMapping.getMapping(event) == XBoxOneMapping.A)
      {
         QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_STEPPING);
         packetCommunicator.send(eventPacket);
      }

      if(XBoxOneMapping.getMapping(event) == XBoxOneMapping.X)
      {
         xGaitRequested.set(true);
      }

      switch (XBoxOneMapping.getMapping(event))
      {
      case RIGHT_BUMPER:
         xGaitSettings.setStepDuration(xGaitStepDuration[0].getValue());
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration[0].getValue());
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift[0].getValue());
         break;
      case LEFT_BUMPER:
         xGaitSettings.setStepDuration(xGaitStepDuration[1].getValue());
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration[1].getValue());
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift[1].getValue());
         break;
      }
   }

   @Override
   public void onExit()
   {

   }

   private boolean isInStepState()
   {
      QuadrupedForceControllerStatePacket forceControllerStatePacket = this.forceControlStatePacket.get();
      QuadrupedSteppingStatePacket steppingStatePacket = this.steppingStatePacket.get();
      return (forceControllerStatePacket != null && forceControllerStatePacket.get() == QuadrupedForceControllerEnum.STEPPING) && (steppingStatePacket != null
            && steppingStatePacket.get() == QuadrupedSteppingStateEnum.STEP);
   }

   private void sendSteps()
   {
      QuadrupedTimedStepPacket timedStepPacket = new QuadrupedTimedStepPacket(stepStream.getSteps(), true);
      packetCommunicator.send(timedStepPacket);
   }
}
