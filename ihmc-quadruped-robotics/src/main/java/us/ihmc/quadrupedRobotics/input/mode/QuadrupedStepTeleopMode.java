package us.ihmc.quadrupedRobotics.input.mode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;

import net.java.games.input.Event;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitPlanner;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedStepTeleopMode implements QuadrupedTeleopMode
{
   private static final double DT = 0.01;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleParameter yawScaleParameter = new DoubleParameter("yawScale", registry, 0.15);
   private final DoubleParameter pitchScaleParameter = new DoubleParameter("pitchScale", registry, 0.15);
   private final DoubleParameter zdotScaleParameter = new DoubleParameter("zdotScale", registry, 0.25);
   private final DoubleParameter defaultGroundClearance = new DoubleParameter("defaultGroundClearance", registry, 0.075);
   private final DoubleParameter xStrideMax = new DoubleParameter("xStrideMax", registry, 0.4);
   private final DoubleParameter yStrideMax = new DoubleParameter("yStrideMax", registry, 0.25);
   private final DoubleParameter yawRateScale = new DoubleParameter("yawRateScale", registry, 0.25);

   // single step parameters
   private final DoubleParameter singleStepShiftDuration = new DoubleParameter("singleStepShiftDuration", registry, 1.0);
   private final DoubleParameter singleStepSwingDuration = new DoubleParameter("singleStepSwingDuration", registry, 2.0);

   // xgait step parameters
   private final IntegerParameter xGaitStepPlanSize = new IntegerParameter("xGaitStepPlanSize", registry, 10);
   private final DoubleParameter[] xGaitStepDuration = new DoubleParameter[2];
   private final DoubleParameter[] xGaitEndDoubleSupportDuration = new DoubleParameter[2];
   private final DoubleParameter[] xGaitEndPhaseShift = new DoubleParameter[2];

   private final PacketCommunicator packetCommunicator;
   private final QuadrupedReferenceFrames referenceFrames;
   private InputValueIntegrator comZ;
   private final QuadrupedXGaitPlanner xGaitStepPlanner = new QuadrupedXGaitPlanner();
   private final YoQuadrupedXGaitSettings xGaitSettings;

   public QuadrupedStepTeleopMode(PacketCommunicator packetCommunicator, QuadrupedPhysicalProperties physicalProperties, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                  QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.packetCommunicator = packetCommunicator;
      this.referenceFrames = referenceFrames;
      this.comZ = new InputValueIntegrator(DT, physicalProperties.getNominalCoMHeight());
      this.xGaitSettings = new YoQuadrupedXGaitSettings(xGaitSettings, null, registry);
      xGaitStepDuration[0] = new DoubleParameter("xGaitStepDurationMode0", registry, 0.5);
      xGaitStepDuration[1] = new DoubleParameter("xGaitStepDurationMode1", registry, 0.33);
      xGaitEndDoubleSupportDuration[0] = new DoubleParameter("xGaitEndDoubleSupportDurationMode0", registry, 1.0);
      xGaitEndDoubleSupportDuration[1] = new DoubleParameter("xGaitEndDoubleSupportDurationMode1", registry, 0.05);
      xGaitEndPhaseShift[0] = new DoubleParameter("xGaitEndPhaseShiftMode0", registry, 90);
      xGaitEndPhaseShift[1] = new DoubleParameter("xGaitEndPhaseShiftMode1", registry, 180);

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
   }

   @Override
   public void onInputEvent(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimates estimates, Event event)
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
         double xStride = xStrideMax.getValue() * channels.get(XBoxOneMapping.LEFT_STICK_Y);
         double yStride = yStrideMax.getValue() * channels.get(XBoxOneMapping.LEFT_STICK_X);
         sendSingleFootstep(quadrantToStep, xStride, yStride);
      }

      Boolean triggeredXGait;
      switch (XBoxOneMapping.getMapping(event))
      {
      case RIGHT_BUMPER:
         triggeredXGait = true;
         xGaitSettings.setStepDuration(xGaitStepDuration[0].getValue());
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration[0].getValue());
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift[0].getValue());
         break;
      case LEFT_BUMPER:
         triggeredXGait = true;
         xGaitSettings.setStepDuration(xGaitStepDuration[1].getValue());
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration[1].getValue());
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift[1].getValue());
         break;
      default:
         triggeredXGait = false;
         break;
      }

      if (triggeredXGait)
      {
         double xVelocityMax = 0.5 * xStrideMax.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double yVelocityMax = 0.5 * yStrideMax.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double yawRateMax = yawRateScale.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double xVelocity = channels.get(XBoxOneMapping.LEFT_STICK_Y) * xVelocityMax;
         double yVelocity = channels.get(XBoxOneMapping.LEFT_STICK_X) * yVelocityMax;
         double yawRate = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawRateMax;
         Vector3D planarVelocity = new Vector3D(xVelocity, yVelocity, yawRate);
         sendXGaitFootsteps(xGaitSettings, planarVelocity, xGaitStepPlanSize.getValue());
      }
   }

   @Override
   public void onExit()
   {

   }

   private void sendSingleFootstep(RobotQuadrant quadrant, double xStride, double yStride)
   {
      // TODO: Compute footstep goal position using more robust method.
      FramePoint3D goalPosition = new FramePoint3D(referenceFrames.getFootFrame(quadrant));
      goalPosition.changeFrame(referenceFrames.getBodyZUpFrame());
      goalPosition.add(xStride, yStride, 0.0);
      goalPosition.changeFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
      goalPosition.setZ(0.0);
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

      final double groundClearance = defaultGroundClearance.getValue();
      TimeInterval timeInterval = new TimeInterval(0.0, singleStepSwingDuration.getValue());
      timeInterval.shiftInterval(singleStepShiftDuration.getValue());

      QuadrupedTimedStep step = new QuadrupedTimedStep(quadrant, goalPosition, groundClearance, timeInterval);
      List<QuadrupedTimedStep> steps = Collections.singletonList(step);
      QuadrupedTimedStepPacket timedStepPacket = new QuadrupedTimedStepPacket(steps, false);
      packetCommunicator.send(timedStepPacket);
   }

   private void sendXGaitFootsteps(QuadrupedXGaitSettingsReadOnly xGaitSettings, Vector3D planarVelocity, int numberOfSteps)
   {
      ReferenceFrame supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint3D supportCentroid = new FramePoint3D();
      supportCentroid.setToZero(supportFrame);
      supportCentroid.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion supportOrientation = new FrameQuaternion();
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
         steps.get(i).setGroundClearance(defaultGroundClearance.getValue());
      }

      QuadrupedTimedStepPacket timedStepPacket = new QuadrupedTimedStepPacket(steps, false);
      packetCommunicator.send(timedStepPacket);
   }
}
