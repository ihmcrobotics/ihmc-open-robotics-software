package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TransferCoPTrajectory
{
   private final YoDouble transferDuration;
   private final YoDouble transferSplitFraction;

   private final YoFramePolynomial3D transferSegment;
   private final FramePoint intermediatePoint = new FramePoint();

   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   public TransferCoPTrajectory(String namePrefix, int stepNumber, YoDouble transferDuration, YoDouble transferSplitFraction, YoVariableRegistry registry)
   {
      this.transferDuration = transferDuration;
      this.transferSplitFraction = transferSplitFraction;

      transferSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "TransferSegment", 5, referenceFrame, registry);
   }

   public void compute(CoPSplineType splineType, FramePoint initialCoP, FramePoint finalCoP)
   {
      intermediatePoint.interpolate(initialCoP, finalCoP, transferSplitFraction.getDoubleValue());
      double intermediateDuration = 0.5 * transferDuration.getDoubleValue();

      switch (splineType)
      {
      case CUBIC:
         transferSegment.setCubicUsingIntermediatePoint(0.0, intermediateDuration, transferDuration.getDoubleValue(), initialCoP, intermediatePoint, finalCoP);
         break;
      default:
         transferSegment.setLinearWithIntermediatePoint(0.0, intermediateDuration, transferDuration.getDoubleValue(), initialCoP, intermediatePoint, finalCoP);
         break;
      }
   }
}
