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

public class TransferCoPTrajectory
{
   private final YoDouble transferDuration;
   private final YoDouble transferSplitFraction;

   private final YoInteger numberOfSegments;

   private final YoFramePolynomial3D transferInitialSegment;
   private final YoFramePolynomial3D transferEndSegment;

   private final FramePoint intermediatePoint = new FramePoint();

   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   private final List<YoFramePolynomial3D> transferSegments = new ArrayList<>();

   public TransferCoPTrajectory(String namePrefix, int stepNumber, YoDouble transferDuration, YoDouble transferSplitFraction, YoVariableRegistry registry)
   {
      this.transferDuration = transferDuration;
      this.transferSplitFraction = transferSplitFraction;

      numberOfSegments = new YoInteger(namePrefix + stepNumber + "TransferNumberOfSegments", registry);

      transferInitialSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "TransferInitialSegment", 5, referenceFrame, registry);
      transferEndSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "TransferEndSegment", 5, referenceFrame, registry);
   }

   public void reset()
   {
      transferInitialSegment.reset();
      transferEndSegment.reset();

      transferSegments.clear();
   }

   public void update(CoPSplineType splineType, FramePoint initialCoP, FramePoint finalCoP)
   {
      intermediatePoint.interpolate(initialCoP, finalCoP, transferSplitFraction.getDoubleValue());
      double intermediateDuration = 0.5 * transferDuration.getDoubleValue();

      switch (splineType)
      {
      case CUBIC:
         transferInitialSegment.setCubicUsingIntermediatePoint(0.0, intermediateDuration, transferDuration.getDoubleValue(), initialCoP, intermediatePoint, finalCoP);

         transferSegments.add(transferInitialSegment);

         numberOfSegments.set(1);
         break;
      default:
         transferInitialSegment.setLinear(0.0, intermediateDuration, initialCoP, intermediatePoint);
         transferEndSegment.setLinear(intermediateDuration, transferDuration.getDoubleValue(), intermediatePoint, finalCoP);

         transferSegments.add(transferInitialSegment);
         transferSegments.add(transferEndSegment);

         numberOfSegments.set(2);
         break;
      }
   }
}
