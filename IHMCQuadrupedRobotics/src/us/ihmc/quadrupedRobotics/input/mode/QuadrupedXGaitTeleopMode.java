package us.ihmc.quadrupedRobotics.input.mode;

import java.util.Map;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.InputChannel;
import us.ihmc.quadrupedRobotics.input.InputEvent;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.packets.PlanarVelocityPacket;
import us.ihmc.quadrupedRobotics.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;

public class QuadrupedXGaitTeleopMode implements QuadrupedTeleopMode
{
   private static final double DT = 0.01;

   private enum XGaitInputMode
   {
      POSITION, VELOCITY, STEP
   }

   private final ParameterFactory parameterFactory = new ParameterFactory(getClass());
   private final DoubleParameter rollScaleParameter = parameterFactory.createDouble("paramRollScale", 0.15);
   private final DoubleParameter pitchScaleParameter = parameterFactory.createDouble("paramPitchScale", 0.15);
   private final DoubleParameter yawScaleParameter = parameterFactory.createDouble("paramYawScale", 0.15);
   private final DoubleParameter xScaleParameter = parameterFactory.createDouble("paramXScale", 0.20);
   private final DoubleParameter yScaleParameter = parameterFactory.createDouble("paramYScale", 0.10);
   private final DoubleParameter vxScaleParameter = parameterFactory.createDouble("paramVxScale", 1.0);
   private final DoubleParameter vyScaleParameter = parameterFactory.createDouble("paramVyScale", 0.5);
   private final DoubleParameter vzScaleParameter = parameterFactory.createDouble("paramVzScale", 0.25);
   private final DoubleParameter wzScaleParameter = parameterFactory.createDouble("paramWzScale", 1.0);
   private final DoubleParameter defaultComHeightParameter = parameterFactory.createDouble("paramDefaultComHeight", 0.55);

   private final PacketCommunicator packetCommunicator;
   private final QuadrupedReferenceFrames referenceFrames;

   private XGaitInputMode mode = XGaitInputMode.POSITION;
   private InputValueIntegrator comZ;

   public QuadrupedXGaitTeleopMode(PacketCommunicator packetCommunicator, QuadrupedReferenceFrames referenceFrames)
   {
      this.packetCommunicator = packetCommunicator;
      this.referenceFrames = referenceFrames;
      this.comZ = new InputValueIntegrator(DT, defaultComHeightParameter.get());
   }

   @Override
   public void onEntry()
   {
      QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
      packetCommunicator.send(eventPacket);
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
      if (mode == XGaitInputMode.VELOCITY)
      {
         xVelocity = channels.get(InputChannel.LEFT_STICK_Y) * vxScaleParameter.get();
         yVelocity = channels.get(InputChannel.LEFT_STICK_X) * vyScaleParameter.get();
         yawRate = channels.get(InputChannel.RIGHT_STICK_X) * wzScaleParameter.get();
      }
      PlanarVelocityPacket velocityPacket = new PlanarVelocityPacket(xVelocity, yVelocity, yawRate);
      packetCommunicator.send(velocityPacket);

      double comX = 0.0;
      double comY = 0.0;
      double comZdot = (channels.get(InputChannel.RIGHT_BUTTON) - channels.get(InputChannel.LEFT_BUTTON)) * vzScaleParameter.get();
      if (mode == XGaitInputMode.POSITION)
      {
         comX = channels.get(InputChannel.LEFT_STICK_Y) * xScaleParameter.get();
      }
      ComPositionPacket comPositionPacket = new ComPositionPacket(comX, comY, comZ.update(comZdot));
      packetCommunicator.send(comPositionPacket);
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
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_TROT);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
         break;
      case BUTTON_Y:
         if (channels.get(InputChannel.BUTTON_Y) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
         break;
      case BUTTON_B:
         if (channels.get(InputChannel.BUTTON_B) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_PACE);
            packetCommunicator.send(eventPacket);
            mode = XGaitInputMode.VELOCITY;
         }
      }
   }

   @Override
   public void onExit()
   {

   }
}
