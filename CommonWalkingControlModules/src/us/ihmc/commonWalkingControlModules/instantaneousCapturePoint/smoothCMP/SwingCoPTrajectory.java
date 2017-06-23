package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class SwingCoPTrajectory
{
   private final YoDouble swingDuration;
   private final YoDouble swingSplitFraction;
   private final YoDouble swingShiftDurationFraction;

   private final YoFramePolynomial3D swingShiftSegment;
   private final YoFramePolynomial3D swingToeShiftSegment;
   private final YoFramePolynomial3D swingConstantSegment;

   private final List<YoFramePolynomial3D> swingSegments = new ArrayList<>();

   private final FramePoint intermediatePoint = new FramePoint();

   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   public SwingCoPTrajectory(String namePrefix, int stepNumber, YoDouble swingDuration, YoDouble swingSplitFraction,
                             YoDouble swingShiftDurationFraction, YoVariableRegistry registry)
   {
      this.swingDuration = swingDuration;
      this.swingSplitFraction = swingSplitFraction;
      this.swingShiftDurationFraction = swingShiftDurationFraction;

      swingShiftSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "SwingShiftSegment", 5, referenceFrame, registry);
      swingToeShiftSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "SwingToeShiftSegment", 5, referenceFrame, registry);
      swingConstantSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "SwingConstantSegment", 1, referenceFrame, registry);
   }

   public void computeOnePointPerFoot(FramePoint cop)
   {
      swingConstantSegment.setConstant(cop);
      swingSegments.add(swingConstantSegment);
   }

   public void updateTwoPointsPerFoot(CoPSplineType splineType, FramePoint heelCoP, FramePoint ballCoP)
   {
      intermediatePoint.interpolate(heelCoP, ballCoP, swingSplitFraction.getDoubleValue());

      double shiftDuration = swingShiftDurationFraction.getDoubleValue() * swingDuration.getDoubleValue();
      double constantDuration = (1.0 - swingShiftDurationFraction.getDoubleValue()) * swingDuration.getDoubleValue();
      double intermediateDuration = 0.5 * shiftDuration;

      switch (splineType)
      {
      case CUBIC:
         swingShiftSegment.setCubicUsingIntermediatePoint(0.0, intermediateDuration, constantDuration, heelCoP, intermediatePoint, ballCoP);
         break;
      default:
         swingShiftSegment.setLinearWithIntermediatePoint(0.0, intermediateDuration, constantDuration, heelCoP, intermediatePoint, ballCoP);
         break;
      }

      swingConstantSegment.setConstant(ballCoP);

      swingSegments.add(swingShiftSegment);
      swingSegments.add(swingConstantSegment);
   }

   public void updateThreePointsPerFoot(CoPSplineType splineType, FramePoint heelCoP, FramePoint ballCoP, FramePoint toeCoP)
   {
      double shiftDuration = swingShiftDurationFraction.getDoubleValue() * swingDuration.getDoubleValue();
      double constantDuration = (1.0 - swingShiftDurationFraction.getDoubleValue()) * swingDuration.getDoubleValue();
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

      swingConstantSegment.setConstant(toeCoP);

      swingSegments.add(swingShiftSegment);
      swingSegments.add(swingToeShiftSegment);
      swingSegments.add(swingConstantSegment);
   }

}
