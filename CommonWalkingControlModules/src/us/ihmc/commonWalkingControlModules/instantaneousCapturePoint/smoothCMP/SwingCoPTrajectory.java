package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SwingCoPTrajectory
{
   private final YoDouble swingDuration;
   private final YoDouble swingSplitFraction;
   private final YoDouble swingShiftDurationFraction;

   private final YoFramePolynomial3D swingShiftSegment;
   private final YoFramePolynomial3D swingConstantSegment;
   private final FramePoint intermediatePoint = new FramePoint();

   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   public SwingCoPTrajectory(String namePrefix, int stepNumber, YoDouble swingDuration, YoDouble swingSplitFraction, YoDouble swingShiftDurationFraction,
                             YoVariableRegistry registry)
   {
      this.swingDuration = swingDuration;
      this.swingSplitFraction = swingSplitFraction;
      this.swingShiftDurationFraction = swingShiftDurationFraction;

      swingShiftSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "SwingShiftSegment", 5, referenceFrame, registry);
      swingConstantSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "SwingConstantSegment", 1, referenceFrame, registry);
   }

   public void compute(CoPSplineType splineType, FramePoint initialCoP, FramePoint finalCoP)
   {
      intermediatePoint.interpolate(initialCoP, finalCoP, swingSplitFraction.getDoubleValue());

      double shiftDuration = swingShiftDurationFraction.getDoubleValue() * swingDuration.getDoubleValue();
      double constantDuration = (1.0 - swingShiftDurationFraction.getDoubleValue()) * swingDuration.getDoubleValue();
      double intermediateDuration = 0.5 * shiftDuration;

      switch (splineType)
      {
      case CUBIC:
         swingShiftSegment.setCubicUsingIntermediatePoint(0.0, intermediateDuration, constantDuration, initialCoP, intermediatePoint, finalCoP);
         break;
      default:
         swingShiftSegment.setLinearWithIntermediatePoint(0.0, intermediateDuration, constantDuration, initialCoP, intermediatePoint, finalCoP);
         break;
      }

      swingConstantSegment.setConstant(finalCoP);
   }
}
