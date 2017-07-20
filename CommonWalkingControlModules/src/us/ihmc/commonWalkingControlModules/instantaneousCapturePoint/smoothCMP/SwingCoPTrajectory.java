package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SwingCoPTrajectory implements CoPTrajectory
{
   private final YoDouble swingDuration;
   private final YoDouble swingSplitFraction;
   private final YoDouble swingDurationShiftFraction;

   private final YoInteger numberOfSegments;
   private final YoInteger currentSegment;

   private final YoFrameTrajectory3D swingShiftSegment;
   private final YoFrameTrajectory3D swingToeShiftSegment;
   private final YoFrameTrajectory3D swingConstantSegment;

   private final List<YoFrameTrajectory3D> swingSegments = new ArrayList<>();

   private final FramePoint intermediatePoint = new FramePoint();

   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   public SwingCoPTrajectory(String namePrefix, int stepNumber, YoDouble swingDuration, YoDouble swingSplitFraction,
                             YoDouble swingDurationShiftFraction, YoVariableRegistry registry)
   {
      this.swingDuration = swingDuration;
      this.swingSplitFraction = swingSplitFraction;
      this.swingDurationShiftFraction = swingDurationShiftFraction;

      numberOfSegments = new YoInteger(namePrefix + stepNumber + "SwingNumberOfSegments", registry);
      currentSegment = new YoInteger(namePrefix + stepNumber + "SwingCurrentSegment", registry);

      swingShiftSegment = new YoFrameTrajectory3D(namePrefix + stepNumber + "SwingShiftSegment", 5, referenceFrame, registry);
      swingToeShiftSegment = new YoFrameTrajectory3D(namePrefix + stepNumber + "SwingToeShiftSegment", 5, referenceFrame, registry);
      swingConstantSegment = new YoFrameTrajectory3D(namePrefix + stepNumber + "SwingConstantSegment", 1, referenceFrame, registry);
   }

   @Override
   public void reset()
   {
      swingShiftSegment.reset();
      swingToeShiftSegment.reset();
      swingConstantSegment.reset();

      swingSegments.clear();
      currentSegment.set(-1);
   }

   private final FramePoint copToThrowAway = new FramePoint();
   private final FrameVector copVelocityToThrowAway = new FrameVector();
   private final FrameVector copAccelerationToThrowAway = new FrameVector();

   @Override
   public void update(double timeInState)
   {
      update(timeInState, copToThrowAway, copVelocityToThrowAway, copAccelerationToThrowAway);
   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack)
   {
      update(timeInState, desiredCoPToPack, copVelocityToThrowAway, copAccelerationToThrowAway);
   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack)
   {
      update(timeInState, desiredCoPToPack, desiredCoPVelocityToPack, copAccelerationToThrowAway);
   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack, FrameVector desiredCoPAccelerationToPack)
   {
      if (numberOfSegments.getIntegerValue() > 2)
      {
         if (swingShiftSegment.timeIntervalContains(timeInState))
         {
            currentSegment.set(1);
            swingShiftSegment.compute(timeInState);
            swingShiftSegment.getFramePosition(desiredCoPToPack);
            swingShiftSegment.getFrameVelocity(desiredCoPVelocityToPack);
            swingShiftSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
         }
         else if (swingToeShiftSegment.timeIntervalContains(timeInState))
         {
            currentSegment.set(2);
            swingToeShiftSegment.compute(timeInState);
            swingToeShiftSegment.getFramePosition(desiredCoPToPack);
            swingToeShiftSegment.getFrameVelocity(desiredCoPVelocityToPack);
            swingToeShiftSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
         }
         else
         {
            currentSegment.set(3);
            swingConstantSegment.compute(timeInState);
            swingConstantSegment.getFramePosition(desiredCoPToPack);
            swingConstantSegment.getFrameVelocity(desiredCoPVelocityToPack);
            swingConstantSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
         }
      }
      else if (numberOfSegments.getIntegerValue() > 1)
      {
         if (swingShiftSegment.timeIntervalContains(timeInState))
         {
            currentSegment.set(1);
            swingShiftSegment.compute(timeInState);
            swingShiftSegment.getFramePosition(desiredCoPToPack);
            swingShiftSegment.getFrameVelocity(desiredCoPVelocityToPack);
            swingShiftSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
         }
         else
         {
            currentSegment.set(2);
            swingConstantSegment.compute(timeInState);
            swingConstantSegment.getFramePosition(desiredCoPToPack);
            swingConstantSegment.getFrameVelocity(desiredCoPVelocityToPack);
            swingConstantSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
         }
      }
      else
      {
         currentSegment.set(1);
         swingConstantSegment.compute(timeInState);
         swingConstantSegment.getFramePosition(desiredCoPToPack);
         swingConstantSegment.getFrameVelocity(desiredCoPVelocityToPack);
         swingConstantSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
      }
   }

   @Override
   public List<YoFrameTrajectory3D> getPolynomials()
   {
      return swingSegments;
   }

   @Override
   public int getNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }

   public void computeOnePointPerFoot(FramePoint cop)
   {
      swingConstantSegment.setConstant(0.0, swingDuration.getDoubleValue(), cop);
      swingSegments.add(swingConstantSegment);

      numberOfSegments.set(1);
   }

   public void computeTwoPointsPerFoot(CoPSplineType splineType, FramePoint heelCoP, FramePoint ballCoP)
   {
      intermediatePoint.interpolate(heelCoP, ballCoP, swingSplitFraction.getDoubleValue());

      double shiftDuration = swingDurationShiftFraction.getDoubleValue() * swingDuration.getDoubleValue();
      double intermediateDuration = 0.5 * shiftDuration;

      switch (splineType)
      {
      case CUBIC:
         swingShiftSegment.setCubicUsingIntermediatePoint(0.0, intermediateDuration, shiftDuration, heelCoP, intermediatePoint, ballCoP);
         swingConstantSegment.setConstant(shiftDuration, swingDuration.getDoubleValue(), ballCoP);

         swingSegments.add(swingShiftSegment);
         swingSegments.add(swingConstantSegment);

         numberOfSegments.set(2);
         break;
      default:
         swingShiftSegment.setLinear(0.0, intermediateDuration, heelCoP, intermediatePoint);
         swingToeShiftSegment.setLinear(intermediateDuration, shiftDuration,  intermediatePoint, ballCoP);
         swingConstantSegment.setConstant(shiftDuration, swingDuration.getDoubleValue(), ballCoP);

         swingSegments.add(swingShiftSegment);
         swingSegments.add(swingToeShiftSegment);
         swingSegments.add(swingConstantSegment);

         numberOfSegments.set(3);
         break;
      }

   }

   public void computeThreePointsPerFoot(CoPSplineType splineType, FramePoint heelCoP, FramePoint ballCoP, FramePoint toeCoP)
   {
      double shiftDuration = swingDurationShiftFraction.getDoubleValue() * swingDuration.getDoubleValue();
      double constantDuration = (1.0 - swingDurationShiftFraction.getDoubleValue()) * swingDuration.getDoubleValue();
      double intermediateDuration = swingSplitFraction.getDoubleValue() * shiftDuration;

      switch (splineType)
      {
      case CUBIC:
         swingShiftSegment.setCubic(0.0, intermediateDuration, heelCoP, ballCoP);
         swingToeShiftSegment.setCubic(intermediateDuration, shiftDuration, ballCoP, toeCoP);
         break;
      default:
         swingShiftSegment.setLinear(0.0, constantDuration, heelCoP, ballCoP);
         swingToeShiftSegment.setLinear(intermediateDuration, shiftDuration, ballCoP, toeCoP);
         break;
      }

      swingConstantSegment.setConstant(shiftDuration, swingDuration.getDoubleValue(), toeCoP);

      swingSegments.add(swingShiftSegment);
      swingSegments.add(swingToeShiftSegment);
      swingSegments.add(swingConstantSegment);

      numberOfSegments.set(3);
   }

}
