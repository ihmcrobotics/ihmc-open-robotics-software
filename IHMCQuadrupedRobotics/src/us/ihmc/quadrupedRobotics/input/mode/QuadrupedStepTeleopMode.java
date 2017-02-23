package us.ihmc.quadrupedRobotics.input.mode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.tuple3D.Vector3D;

import net.java.games.input.Event;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator.Estimates;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.IntegerParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitPlanner;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public class QuadrupedStepTeleopMode implements QuadrupedTeleopMode
{
   private static final double DT = 0.01;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithoutRegistry(getClass());
   private final DoubleParameter yawScaleParameter = parameterFactory.createDouble("yawScale", 0.15);
   private final DoubleParameter pitchScaleParameter = parameterFactory.createDouble("pitchScale", 0.15);
   private final DoubleParameter zdotScaleParameter = parameterFactory.createDouble("zdotScale", 0.25);
   private final DoubleParameter defaultComHeightParameter = parameterFactory.createDouble("defaultComHeight", 0.6);
   private final DoubleParameter defaultGroundClearance = parameterFactory.createDouble("defaultGroundClearance", 0.075);
   private final DoubleParameter xStrideMax = parameterFactory.createDouble("xStrideMax", 0.4);
   private final DoubleParameter yStrideMax = parameterFactory.createDouble("yStrideMax", 0.25);
   private final DoubleParameter yawRateScale = parameterFactory.createDouble("yawRateScale", 0.25);

   // single step parameters
   private final DoubleParameter singleStepShiftDuration = parameterFactory.createDouble("singleStepShiftDuration", 1.0);
   private final DoubleParameter singleStepSwingDuration = parameterFactory.createDouble("singleStepSwingDuration", 2.0);

   // xgait step parameters
   private final IntegerParameter xGaitStepPlanSize = parameterFactory.createInteger("xGaitStepPlanSize", 10);
   private final DoubleParameter xGaitStanceWidth = parameterFactory.createDouble("xGaitStanceWidth", 0.25);
   private final DoubleParameter xGaitStanceLength = parameterFactory.createDouble("xGaitStanceWidth", 1.0);
   private final DoubleArrayParameter xGaitStepDuration = parameterFactory.createDoubleArray("xGaitStepDuration", 0.5, 0.33);
   private final DoubleArrayParameter xGaitEndDoubleSupportDuration = parameterFactory.createDoubleArray("xGaitEndDoubleSupportDuration", 1.0, 0.05);
   private final DoubleArrayParameter xGaitEndPhaseShift = parameterFactory.createDoubleArray("xGaitEndPhaseShift", 90, 180);

   private final PacketCommunicator packetCommunicator;
   private final QuadrupedReferenceFrames referenceFrames;
   private InputValueIntegrator comZ;
   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final QuadrupedXGaitSettings xGaitSettings;

   public QuadrupedStepTeleopMode(PacketCommunicator packetCommunicator, QuadrupedReferenceFrames referenceFrames)
   {
      this.packetCommunicator = packetCommunicator;
      this.referenceFrames = referenceFrames;
      this.comZ = new InputValueIntegrator(DT, defaultComHeightParameter.get());
      this.xGaitStepPlanner = new QuadrupedXGaitPlanner();
      this.xGaitSettings = new QuadrupedXGaitSettings();
      this.xGaitSettings.setStanceWidth(xGaitStanceWidth.get());
      this.xGaitSettings.setStanceLength(xGaitStanceLength.get());
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void update(Map<XBoxOneMapping, Double> channels, Estimates estimates)
   {
      double bodyRoll = 0.0;
      double bodyPitch = channels.get(XBoxOneMapping.RIGHT_STICK_Y) * pitchScaleParameter.get();
      double bodyYaw = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawScaleParameter.get();
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(bodyYaw, bodyPitch, bodyRoll);
      packetCommunicator.send(orientationPacket);
   
      double comZdot = 0.0;
      if (channels.get(XBoxOneMapping.DPAD) == 0.25)
      {
         comZdot += zdotScaleParameter.get();
      }
      if (channels.get(XBoxOneMapping.DPAD) == 0.75)
      {
         comZdot -= zdotScaleParameter.get();
      }
      ComPositionPacket comPositionPacket = new ComPositionPacket(0.0, 0.0, comZ.update(comZdot));
      packetCommunicator.send(comPositionPacket);
   }

   @Override
   public void onInputEvent(Map<XBoxOneMapping, Double> channels, Estimates estimates, Event event)
   {
      if (event.getValue() < 0.5)
         return;

      // Each button steps a different foot. The step length is determined by the left stick forward/back.
      RobotQuadrant quadrantToStep;
      switch (XBoxOneMapping.getMapping(event))
      {
      case X:
         quadrantToStep = RobotQuadrant.FRONT_LEFT;
         break;
      case Y:
         quadrantToStep = RobotQuadrant.FRONT_RIGHT;
         break;
      case B:
         quadrantToStep = RobotQuadrant.HIND_RIGHT;
         break;
      case A:
         quadrantToStep = RobotQuadrant.HIND_LEFT;
         break;
      default:
         quadrantToStep = null;
         break;
      }
   
      if (quadrantToStep != null)
      {
         double xStride = xStrideMax.get() * channels.get(XBoxOneMapping.LEFT_STICK_Y);
         double yStride = yStrideMax.get() * channels.get(XBoxOneMapping.LEFT_STICK_X);
         sendSingleFootstep(quadrantToStep, xStride, yStride);
      }
   
      Boolean triggeredXGait;
      switch (XBoxOneMapping.getMapping(event))
      {
      case RIGHT_BUMPER:
         triggeredXGait = true;
         xGaitSettings.setStepDuration(xGaitStepDuration.get(0));
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration.get(0));
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift.get(0));
         break;
      case LEFT_BUMPER:
         triggeredXGait = true;
         xGaitSettings.setStepDuration(xGaitStepDuration.get(1));
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration.get(1));
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift.get(1));
         break;
      default:
         triggeredXGait = false;
         break;
      }
   
      if (triggeredXGait)
      {
         double xVelocityMax = 0.5 * xStrideMax.get() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double yVelocityMax = 0.5 * yStrideMax.get() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double yawRateMax = yawRateScale.get() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double xVelocity = channels.get(XBoxOneMapping.LEFT_STICK_Y) * xVelocityMax;
         double yVelocity = channels.get(XBoxOneMapping.LEFT_STICK_X) * yVelocityMax;
         double yawRate = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawRateMax;
         Vector3D planarVelocity = new Vector3D(xVelocity, yVelocity, yawRate);
         sendXGaitFootsteps(xGaitSettings, planarVelocity, xGaitStepPlanSize.get());
      }
   }

   @Override
   public void onExit()
   {

   }

   private void sendSingleFootstep(RobotQuadrant quadrant, double xStride, double yStride)
   {
      // TODO: Compute footstep goal position using more robust method.
      FramePoint goalPosition = new FramePoint(referenceFrames.getFootFrame(quadrant));
      goalPosition.changeFrame(referenceFrames.getBodyZUpFrame());
      goalPosition.add(xStride, yStride, 0.0);
      goalPosition.changeFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
      goalPosition.setZ(0.0);
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

      final double groundClearance = defaultGroundClearance.get();
      TimeInterval timeInterval = new TimeInterval(0.0, singleStepSwingDuration.get());
      timeInterval.shiftInterval(singleStepShiftDuration.get());

      QuadrupedTimedStep step = new QuadrupedTimedStep(quadrant, goalPosition.getPoint(), groundClearance, timeInterval);
      List<QuadrupedTimedStep> steps = Collections.singletonList(step);
      QuadrupedTimedStepPacket timedStepPacket = new QuadrupedTimedStepPacket(steps, false);
      packetCommunicator.send(timedStepPacket);
   }

   private void sendXGaitFootsteps(QuadrupedXGaitSettings xGaitSettings, Vector3D planarVelocity, int numberOfSteps)
   {
      ReferenceFrame supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint supportCentroid = new FramePoint();
      supportCentroid.setToZero(supportFrame);
      supportCentroid.changeFrame(ReferenceFrame.getWorldFrame());
      FrameOrientation supportOrientation = new FrameOrientation();
      supportOrientation.setToZero(supportFrame);
      supportOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      double supportYaw = supportOrientation.getYaw();

      double initialStepStartTime = 1.0;
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      ArrayList<QuadrupedTimedStep> steps = new ArrayList<>();
      for (int i = 0; i < numberOfSteps; i++)
      {
         steps.add(new QuadrupedTimedStep());
      }
      xGaitStepPlanner.computeInitialPlan(steps, planarVelocity, initialQuadrant, supportCentroid, initialStepStartTime, supportYaw, xGaitSettings);
      for (int i = 0; i < numberOfSteps; i++)
      {
         steps.get(i).setGroundClearance(defaultGroundClearance.get());
      }

      QuadrupedTimedStepPacket timedStepPacket = new QuadrupedTimedStepPacket(steps, false);
      packetCommunicator.send(timedStepPacket);
   }
}
