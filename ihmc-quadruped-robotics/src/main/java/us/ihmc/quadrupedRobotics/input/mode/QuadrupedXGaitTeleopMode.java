package us.ihmc.quadrupedRobotics.input.mode;

import java.util.Map;

import net.java.games.input.Event;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.packets.*;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.commons.MathTools;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedXGaitTeleopMode implements QuadrupedTeleopMode
{
   private static final double DT = 0.01;

   private enum XGaitInputMode
   {
      POSITION, VELOCITY, STEP
   }

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final double defaultComHeightParameter = 0.60;
   private static final double defaultStanceLengthParameter = 1.0;
   private static final double defaultStanceWidthParameter = 0.35;
   private static final double defaultStepGroundClearanceParameter = 0.075;
   private static final double defaultStepDurationParameter = 0.35;
   private static final double defaultEndDoubleSupportDurationParameter = 0.1;
   private static final double defaultEndPhaseShiftParameter = 180;

   private final DoubleParameter xScaleParameter = new DoubleParameter("xScale", registry, 0.20);
   private final DoubleParameter rollScaleParameter = new DoubleParameter("rollScale", registry, 0.15);
   private final DoubleParameter pitchScaleParameter = new DoubleParameter("pitchScale", registry, 0.15);
   private final DoubleParameter yawScaleParameter = new DoubleParameter("yawScale", registry, 0.15);
   private final DoubleParameter yawRateScaleParameter = new DoubleParameter("yawRateScale", registry, 0.25);
   private final DoubleParameter zRateParameter = new DoubleParameter("zRate", registry, 0.1);
   private final DoubleParameter xStrideMaxParameter = new DoubleParameter("xStrideMax", registry, 0.3);
   private final DoubleParameter yStrideMaxParameter = new DoubleParameter("yStrideMax", registry, 0.15);
   private final DoubleParameter deltaPhaseShiftParameter = new DoubleParameter("deltaPhaseShift", registry, 22.5);
   private final DoubleParameter deltaDoubleSupportParameter = new DoubleParameter("deltaDoubleSupport", registry, 0.0025);
   private final DoubleParameter deltaStanceWidthParameter = new DoubleParameter("deltaStanceWidth", registry, 0.05);

   private final PacketCommunicator packetCommunicator;
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedXGaitSettings xGaitSettings;

   private XGaitInputMode mode = XGaitInputMode.POSITION;
   private InputValueIntegrator comZ;

   public QuadrupedXGaitTeleopMode(PacketCommunicator packetCommunicator, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.packetCommunicator = packetCommunicator;
      this.referenceFrames = referenceFrames;
      this.comZ = new InputValueIntegrator(DT, defaultComHeightParameter);
      this.xGaitSettings = new QuadrupedXGaitSettings();
      this.xGaitSettings.setStanceLength(defaultStanceLengthParameter);
      this.xGaitSettings.setStanceWidth(defaultStanceWidthParameter);
      this.xGaitSettings.setStepGroundClearance(defaultStepGroundClearanceParameter);
      this.xGaitSettings.setStepDuration(defaultStepDurationParameter);
      this.xGaitSettings.setEndDoubleSupportDuration(defaultEndDoubleSupportDurationParameter);
      this.xGaitSettings.setEndPhaseShift(defaultEndPhaseShiftParameter);
      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void update(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimates estimates)
   {
      double bodyYaw = 0.0;
      double bodyRoll = 0.0;
      double bodyPitch = channels.get(XBoxOneMapping.RIGHT_STICK_Y) * pitchScaleParameter.getValue();
      if (mode == XGaitInputMode.POSITION)
      {
         bodyYaw = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawScaleParameter.getValue();
         bodyRoll = -channels.get(XBoxOneMapping.LEFT_STICK_X) * rollScaleParameter.getValue();
      }
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(bodyYaw, bodyPitch, bodyRoll);
      packetCommunicator.send(orientationPacket);


      double xVelocity = 0.0;
      double yVelocity = 0.0;
      double yawRate = 0.0;
      double xVelocityMax = 0.5 * xStrideMaxParameter.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double yVelocityMax = 0.5 * yStrideMaxParameter.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double yawRateMax = yawRateScaleParameter.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      if (mode == XGaitInputMode.VELOCITY)
      {
         xVelocity = channels.get(XBoxOneMapping.LEFT_STICK_Y) * xVelocityMax;
         yVelocity = channels.get(XBoxOneMapping.LEFT_STICK_X) * yVelocityMax;
         yawRate = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawRateMax;
      }
      PlanarVelocityPacket velocityPacket = new PlanarVelocityPacket(xVelocity, yVelocity, yawRate);
      packetCommunicator.send(velocityPacket);

      double comX = 0.0;
      double comY = 0.0;
      double comZdot = 0.0;
      if (channels.get(XBoxOneMapping.DPAD) == 0.25)
      {
         comZdot += zRateParameter.getValue();
      }
      if (channels.get(XBoxOneMapping.DPAD) == 0.75)
      {
         comZdot -= zRateParameter.getValue();
      }
      if (mode == XGaitInputMode.POSITION)
      {
         comX = channels.get(XBoxOneMapping.LEFT_STICK_Y) * xScaleParameter.getValue();
      }
      ComPositionPacket comPositionPacket = new ComPositionPacket(comX, comY, comZ.update(comZdot));
      packetCommunicator.send(comPositionPacket);

      double deltaTrigger = (channels.get(XBoxOneMapping.LEFT_TRIGGER) - channels.get(XBoxOneMapping.RIGHT_TRIGGER));
      xGaitSettings.setEndDoubleSupportDuration(MathTools.clamp(xGaitSettings.getEndDoubleSupportDuration() + deltaTrigger * deltaDoubleSupportParameter.getValue(), 0.1, 1));
      QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
      packetCommunicator.send(settingsPacket);
   }

   @Override
   public void onInputEvent(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimates estimates, Event event)
   {
      switch (XBoxOneMapping.getMapping(event))
      {
      case A:
         if (event.getValue() > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_STEPPING);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.POSITION;
         }
         break;
      case X:
         if (event.getValue() > 0.5)
         {
            xGaitSettings.setEndPhaseShift(180);
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
            QuadrupedSteppingEventPacket eventPacket = new QuadrupedSteppingEventPacket(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
         break;
      case Y:
         if (event.getValue() > 0.5)
         {
            xGaitSettings.setEndPhaseShift(90);
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
            QuadrupedSteppingEventPacket eventPacket = new QuadrupedSteppingEventPacket(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
         break;
      case B:
         if (event.getValue() > 0.5)
         {
            xGaitSettings.setEndPhaseShift(0);
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
            QuadrupedSteppingEventPacket eventPacket = new QuadrupedSteppingEventPacket(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
         break;
      case LEFT_BUMPER:
         if (event.getValue() > 0.5)
         {
            xGaitSettings.setEndPhaseShift(MathTools.clamp(xGaitSettings.getEndPhaseShift() + deltaPhaseShiftParameter.getValue(), 0, 359));
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
         }
         break;
      case RIGHT_BUMPER:
         if (event.getValue() > 0.5)
         {
            xGaitSettings.setEndPhaseShift(MathTools.clamp(xGaitSettings.getEndPhaseShift() - deltaPhaseShiftParameter.getValue(), 0, 359));
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
         }
         break;
      case DPAD:
         if (event.getValue() == 0.5)
         {
            xGaitSettings.setStanceWidth(xGaitSettings.getStanceWidth() + deltaStanceWidthParameter.getValue());
         }
         if (event.getValue() == 1.0)
         {
            xGaitSettings.setStanceWidth(xGaitSettings.getStanceWidth() - deltaStanceWidthParameter.getValue());
         }
         QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
         packetCommunicator.send(settingsPacket);
         break;
      }
   }

   @Override
   public void onExit()
   {

   }
}
