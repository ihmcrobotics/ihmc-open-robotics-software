package us.ihmc.quadrupedRobotics.input.mode;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.InputChannel;
import us.ihmc.quadrupedRobotics.input.InputEvent;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTestTeleopMode implements QuadrupedTeleopMode
{
   private static final double DT = 0.01;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithoutRegistry(getClass());
   private final DoubleParameter rollScaleParameter = parameterFactory.createDouble("rollScale", 0.15);
   private final DoubleParameter pitchScaleParameter = parameterFactory.createDouble("pitchScale", 0.15);
   private final DoubleParameter yawScaleParameter = parameterFactory.createDouble("yawScale", 0.15);
   private final DoubleParameter vzScaleParameter = parameterFactory.createDouble("vzScale", 0.25);
   private final DoubleParameter defaultComHeightParameter = parameterFactory.createDouble("defaultComHeight", 0.55);
   private final DoubleParameter swingHeight = parameterFactory.createDouble("swingHeight", 0.1);
   private final DoubleParameter shiftDuration = parameterFactory.createDouble("shiftDuration", 1.0);
   private final DoubleParameter swingDuration = parameterFactory.createDouble("swingDuration", 2.0);
   private final DoubleParameter swingLengthScale = parameterFactory.createDouble("swingLengthScale", 0.2);

   private final PacketCommunicator packetCommunicator;
   private final QuadrupedReferenceFrames referenceFrames;
   private InputValueIntegrator comZ;

   public QuadrupedTestTeleopMode(PacketCommunicator packetCommunicator, QuadrupedReferenceFrames referenceFrames)
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
      bodyYaw = channels.get(InputChannel.RIGHT_STICK_X) * yawScaleParameter.get();
      bodyRoll = -channels.get(InputChannel.LEFT_STICK_X) * rollScaleParameter.get();
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(bodyYaw, bodyPitch, bodyRoll);
      packetCommunicator.send(orientationPacket);

      double comZdot = 0.0;
      if (channels.get(InputChannel.D_PAD) == 0.25)
      {
         comZdot += vzScaleParameter.get();
      }
      if (channels.get(InputChannel.D_PAD) == 0.75)
      {
         comZdot -= vzScaleParameter.get();
      }
      ComPositionPacket comPositionPacket = new ComPositionPacket(0.0, 0.0, comZ.update(comZdot));
      packetCommunicator.send(comPositionPacket);
   }

   @Override
   public void onInputEvent(Map<InputChannel, Double> channels, QuadrupedTaskSpaceEstimator.Estimates estimates, InputEvent e)
   {
      // Each button steps a different foot. The step length is determined by the left stick forward/back.
      RobotQuadrant quadrantToStep = null;
      switch (e.getChannel())
      {
      case BUTTON_X:
         quadrantToStep = RobotQuadrant.FRONT_LEFT;
         break;
      case BUTTON_Y:
         quadrantToStep = RobotQuadrant.FRONT_RIGHT;
         break;
      case BUTTON_B:
         quadrantToStep = RobotQuadrant.HIND_RIGHT;
         break;
      case BUTTON_A:
         quadrantToStep = RobotQuadrant.HIND_LEFT;
         break;
      }

      if (quadrantToStep != null)
      {
         sendSingleFootstep(quadrantToStep, swingLengthScale.get() * channels.get(InputChannel.LEFT_STICK_Y));
      }
   }

   @Override
   public void onExit()
   {

   }

   private void sendSingleFootstep(RobotQuadrant quadrant, double swingLength)
   {
      // TODO: Compute footstep goal position using more robust method.
      FramePoint goalPosition = new FramePoint(referenceFrames.getFootFrame(quadrant));
      goalPosition.changeFrame(referenceFrames.getBodyZUpFrame());
      goalPosition.add(swingLength, 0.0, 0.0);
      goalPosition.changeFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
      goalPosition.setZ(0.0);
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

      final double groundClearance = swingHeight.get();
      TimeInterval timeInterval = new TimeInterval(0.0, swingDuration.get());
      timeInterval.shiftInterval(shiftDuration.get());

      QuadrupedTimedStep step = new QuadrupedTimedStep(quadrant, goalPosition.getPoint(), groundClearance, timeInterval, false);
      List<QuadrupedTimedStep> steps = Collections.singletonList(step);
      QuadrupedTimedStepPacket timedStepPacket = new QuadrupedTimedStepPacket(steps);
      packetCommunicator.send(timedStepPacket);

      QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_STEP);
      packetCommunicator.send(eventPacket);
   }
}
