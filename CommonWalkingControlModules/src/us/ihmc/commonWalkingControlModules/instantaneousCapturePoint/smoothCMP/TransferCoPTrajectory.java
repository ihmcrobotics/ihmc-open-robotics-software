package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class TransferCoPTrajectory implements CoPTrajectory
{
   private final YoDouble transferDuration;
   private final YoDouble transferSplitFraction;

   private final YoInteger numberOfSegments;
   private final YoInteger currentSegment;

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
      currentSegment = new YoInteger(namePrefix + stepNumber + "TransferCurrentSegment", registry);

      transferInitialSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "TransferInitialSegment", 5, referenceFrame, registry);
      transferEndSegment = new YoFramePolynomial3D(namePrefix + stepNumber + "TransferEndSegment", 5, referenceFrame, registry);
   }

   @Override
   public void reset()
   {
      transferInitialSegment.reset();
      transferEndSegment.reset();

      transferSegments.clear();
      currentSegment.set(-1);
   }

   private final FramePoint copToThrowAway = new FramePoint();
   private final FrameVector copVelocityToThrowAway = new FrameVector();
   private final FrameVector copAccelerationToThrowAway = new FrameVector();

   @Override
   public void update(double timeInState)
   {
      update(timeInState, copToThrowAway);

   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack)
   {
      update(timeInState, desiredCoPToPack, copVelocityToThrowAway);
   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack)
   {
      update(timeInState, desiredCoPToPack, desiredCoPVelocityToPack, copAccelerationToThrowAway);
   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack, FrameVector desiredCoPAccelerationToPack)
   {
      if (numberOfSegments.getIntegerValue() > 1)
      {
         if (transferInitialSegment.timeIntervalContains(timeInState))
         {
            transferInitialSegment.compute(timeInState);
            transferInitialSegment.getFramePosition(desiredCoPToPack);
            transferInitialSegment.getFrameVelocity(desiredCoPVelocityToPack);
            transferInitialSegment.getFrameAcceleration(desiredCoPAccelerationToPack);

            currentSegment.set(1);
         }
         else
         {
            transferEndSegment.compute(timeInState);
            transferEndSegment.getFramePosition(desiredCoPToPack);
            transferEndSegment.getFrameVelocity(desiredCoPVelocityToPack);
            transferEndSegment.getFrameAcceleration(desiredCoPAccelerationToPack);

            currentSegment.set(2);
         }
      }
      else
      {
         transferInitialSegment.compute(timeInState);
         transferInitialSegment.getFramePosition(desiredCoPToPack);
         transferInitialSegment.getFrameVelocity(desiredCoPVelocityToPack);
         transferInitialSegment.getFrameAcceleration(desiredCoPAccelerationToPack);

         currentSegment.set(1);
      }
   }

   @Override
   public List<YoFramePolynomial3D> getPolynomials()
   {
      return transferSegments;
   }

   @Override
   public int getNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }

   public void compute(CoPSplineType splineType, FramePoint initialCoP, FramePoint finalCoP)
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
