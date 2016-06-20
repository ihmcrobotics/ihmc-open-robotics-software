package us.ihmc.quadrupedRobotics.input.mode;

import java.util.Map;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.packets.*;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.InputChannel;
import us.ihmc.quadrupedRobotics.input.InputEvent;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.robotics.MathTools;

public class QuadrupedXGaitTeleopMode implements QuadrupedTeleopMode
{
   private static final double DT = 0.01;

   private enum XGaitInputMode
   {
      POSITION, VELOCITY, STEP
   }

   private final ParameterFactory parameterFactory = ParameterFactory.createWithoutRegistry(getClass());
   private final DoubleParameter xScaleParameter = parameterFactory.createDouble("xScale", 0.20);
   private final DoubleParameter rollScaleParameter = parameterFactory.createDouble("rollScale", 0.15);
   private final DoubleParameter pitchScaleParameter = parameterFactory.createDouble("pitchScale", 0.15);
   private final DoubleParameter yawScaleParameter = parameterFactory.createDouble("yawScale", 0.15);
   private final DoubleParameter yawRateScaleParameter = parameterFactory.createDouble("yawRateScale", 0.25);
   private final DoubleParameter zRateParameter = parameterFactory.createDouble("zRateParameter", 0.5);
   private final DoubleParameter xStrideMaxParameter = parameterFactory.createDouble("xStrideMax", 0.55);
   private final DoubleParameter yStrideMaxParameter = parameterFactory.createDouble("yStrideMax", 0.4);
   private final DoubleParameter deltaPhaseShiftParameter = parameterFactory.createDouble("deltaPhaseShiftParameter", 22.5);
   private final DoubleParameter deltaDoubleSupportParameter = parameterFactory.createDouble("deltaDoubleSupportParameter", 0.0025);
   private final DoubleParameter deltaStanceWidthParameter = parameterFactory.createDouble("deltaStanceWidthParameter", 0.05);
   private final DoubleParameter defaultComHeightParameter = parameterFactory.createDouble("defaultComHeight", 0.55);
   private final DoubleParameter defaultStanceLengthParameter = parameterFactory.createDouble("defaultStanceLength", 1.0);
   private final DoubleParameter defaultStanceWidthParameter = parameterFactory.createDouble("defaultStanceWidth", 0.2);
   private final DoubleParameter defaultStepGroundClearanceParameter = parameterFactory.createDouble("defaultStepGroundClearance", 0.1);
   private final DoubleParameter defaultStepDurationParameter = parameterFactory.createDouble("defaultStepDurationParameter", 0.35);
   private final DoubleParameter defaultEndDoubleSupportDurationParameter = parameterFactory.createDouble("defaultEndDoubleSupportDuration", 0.0);
   private final DoubleParameter defaultEndPhaseShiftParameter = parameterFactory.createDouble("defaultEndPhaseShift", 90);

   private final PacketCommunicator packetCommunicator;
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedXGaitSettings xGaitSettings;

   private XGaitInputMode mode = XGaitInputMode.POSITION;
   private InputValueIntegrator comZ;

   public QuadrupedXGaitTeleopMode(PacketCommunicator packetCommunicator, QuadrupedReferenceFrames referenceFrames)
   {
      this.packetCommunicator = packetCommunicator;
      this.referenceFrames = referenceFrames;
      this.comZ = new InputValueIntegrator(DT, defaultComHeightParameter.get());
      this.xGaitSettings = new QuadrupedXGaitSettings();
      this.xGaitSettings.setStanceLength(defaultStanceLengthParameter.get());
      this.xGaitSettings.setStanceWidth(defaultStanceWidthParameter.get());
      this.xGaitSettings.setStepGroundClearance(defaultStepGroundClearanceParameter.get());
      this.xGaitSettings.setStepDuration(defaultStepDurationParameter.get());
      this.xGaitSettings.setEndDoubleSupportDuration(defaultEndDoubleSupportDurationParameter.get());
      this.xGaitSettings.setEndPhaseShift(defaultEndPhaseShiftParameter.get());
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void update(Map<InputChannel, Double> channels, QuadrupedTaskSpaceEstimator.Estimates estimates)
   {
      double bodyYaw = 0.0;
      double bodyRoll = 0.0;
      double bodyPitch = channels.get(InputChannel.RIGHT_STICK_Y) * pitchScaleParameter.get();
      if (mode == XGaitInputMode.POSITION)
      {
         bodyYaw = channels.get(InputChannel.RIGHT_STICK_X) * yawScaleParameter.get();
         bodyRoll = -channels.get(InputChannel.LEFT_STICK_X) * rollScaleParameter.get();
      }
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(bodyYaw, bodyPitch, bodyRoll);
      packetCommunicator.send(orientationPacket);


      double xVelocity = 0.0;
      double yVelocity = 0.0;
      double yawRate = 0.0;
      double xVelocityMax = 0.5 * xStrideMaxParameter.get() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double yVelocityMax = 0.5 * yStrideMaxParameter.get() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double yawRateMax = yawRateScaleParameter.get() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      if (mode == XGaitInputMode.VELOCITY)
      {
         xVelocity = channels.get(InputChannel.LEFT_STICK_Y) * xVelocityMax;
         yVelocity = channels.get(InputChannel.LEFT_STICK_X) * yVelocityMax;
         yawRate = channels.get(InputChannel.RIGHT_STICK_X) * yawRateMax;
      }
      PlanarVelocityPacket velocityPacket = new PlanarVelocityPacket(xVelocity, yVelocity, yawRate);
      packetCommunicator.send(velocityPacket);

      double comX = 0.0;
      double comY = 0.0;
      double comZdot = 0.0;
      if (channels.get(InputChannel.D_PAD) == 0.25)
      {
         comZdot += zRateParameter.get();
      }
      if (channels.get(InputChannel.D_PAD) == 0.75)
      {
         comZdot -= zRateParameter.get();
      }
      if (mode == XGaitInputMode.POSITION)
      {
         comX = channels.get(InputChannel.LEFT_STICK_Y) * xScaleParameter.get();
      }
      ComPositionPacket comPositionPacket = new ComPositionPacket(comX, comY, comZ.update(comZdot));
      packetCommunicator.send(comPositionPacket);

      double deltaTrigger = (channels.get(InputChannel.LEFT_TRIGGER) - channels.get(InputChannel.RIGHT_TRIGGER));
      xGaitSettings.setEndDoubleSupportDuration(MathTools.clipToMinMax(xGaitSettings.getEndDoubleSupportDuration() + deltaTrigger * deltaDoubleSupportParameter.get(), 0, 1));
      QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
      packetCommunicator.send(settingsPacket);
   }

   @Override
   public void onInputEvent(Map<InputChannel, Double> channels, QuadrupedTaskSpaceEstimator.Estimates estimates, InputEvent e)
   {
      switch (e.getChannel())
      {
      case BUTTON_A:
         if (channels.get(InputChannel.BUTTON_A) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.POSITION;
         }
         break;
      case BUTTON_X:
         if (channels.get(InputChannel.BUTTON_X) > 0.5)
         {
            xGaitSettings.setEndPhaseShift(180);
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
         break;
      case BUTTON_Y:
         if (channels.get(InputChannel.BUTTON_Y) > 0.5)
         {
            xGaitSettings.setEndPhaseShift(90);
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
         break;
      case BUTTON_B:
         if (channels.get(InputChannel.BUTTON_B) > 0.5)
         {
            xGaitSettings.setEndPhaseShift(0);
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
         break;
      case LEFT_BUTTON:
         if (channels.get(InputChannel.LEFT_BUTTON) > 0.5)
         {
            xGaitSettings.setEndPhaseShift(MathTools.clipToMinMax(xGaitSettings.getEndPhaseShift() + deltaPhaseShiftParameter.get(), 0, 359));
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
         }
         break;
      case RIGHT_BUTTON:
         if (channels.get(InputChannel.RIGHT_BUTTON) > 0.5)
         {
            xGaitSettings.setEndPhaseShift(MathTools.clipToMinMax(xGaitSettings.getEndPhaseShift() - deltaPhaseShiftParameter.get(), 0, 359));
            QuadrupedXGaitSettingsPacket settingsPacket = new QuadrupedXGaitSettingsPacket(xGaitSettings);
            packetCommunicator.send(settingsPacket);
         }
         break;
      case D_PAD:
         if (channels.get(InputChannel.D_PAD) == 0.5)
         {
            xGaitSettings.setStanceWidth(xGaitSettings.getStanceWidth() + deltaStanceWidthParameter.get());
         }
         if (channels.get(InputChannel.D_PAD) == 1.0)
         {
            xGaitSettings.setStanceWidth(xGaitSettings.getStanceWidth() - deltaStanceWidthParameter.get());
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
