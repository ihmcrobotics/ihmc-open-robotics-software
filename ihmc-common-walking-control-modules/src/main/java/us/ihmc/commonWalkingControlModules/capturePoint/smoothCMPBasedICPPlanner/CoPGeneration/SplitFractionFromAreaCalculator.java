package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.function.*;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class SplitFractionFromAreaCalculator
{
   private final FramePose3D stanceFootPose = new FramePose3D();
   private final FramePose3D nextFootPose = new FramePose3D();

   private final SplitFractionCalculatorParametersReadOnly splitFractionParameters;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final IntSupplier numberOfStepsProvider;
   private final IntFunction<Footstep> stepGetter;
   private final DoubleSupplier finalTransferWeightDistributionProvider;
   private final DoubleSupplier finalTransferSplitFractionProvider;
   private final IntToDoubleFunction transferWeightDistributionProvider;
   private final IntToDoubleFunction transferSplitFractionProvider;
   private final DoubleConsumer finalTransferWeightDistributionConsumer;
   private final DoubleConsumer finalTransferSplitFractionConsumer;
   private final IntFunction<DoubleConsumer> transferWeightDistributionConsumer;
   private final IntFunction<DoubleConsumer> transferSplitFractionConsumer;

   public SplitFractionFromAreaCalculator(SplitFractionCalculatorParametersReadOnly splitFractionParameters,
                                          SideDependentList<? extends ReferenceFrame> soleFrames,
                                          IntSupplier numberOfStepsProvider,
                                          IntFunction<Footstep> stepGetter,
                                          DoubleSupplier finalTransferWeightDistributionProvider,
                                          DoubleSupplier finalTransferSplitFractionProvider,
                                          IntToDoubleFunction transferWeightDistributionProvider,
                                          IntToDoubleFunction transferSplitFractionProvider,
                                          DoubleConsumer finalTransferWeightDistributionConsumer,
                                          DoubleConsumer finalTransferSplitFractionConsumer,
                                          IntFunction<DoubleConsumer> transferWeightDistributionConsumer,
                                          IntFunction<DoubleConsumer> transferSplitFractionConsumer)
   {
      this.splitFractionParameters = splitFractionParameters;
      this.soleFrames = soleFrames;

      this.numberOfStepsProvider = numberOfStepsProvider;
      this.stepGetter = stepGetter;
      this.finalTransferWeightDistributionProvider = finalTransferWeightDistributionProvider;
      this.finalTransferSplitFractionProvider = finalTransferSplitFractionProvider;
      this.transferWeightDistributionProvider = transferWeightDistributionProvider;
      this.transferSplitFractionProvider = transferSplitFractionProvider;
      this.finalTransferWeightDistributionConsumer = finalTransferWeightDistributionConsumer;
      this.finalTransferSplitFractionConsumer = finalTransferSplitFractionConsumer;
      this.transferWeightDistributionConsumer = transferWeightDistributionConsumer;
      this.transferSplitFractionConsumer = transferSplitFractionConsumer;
   }

   public void computeSplitFractionsFromPosition()
   {
      if (numberOfStepsProvider.getAsInt() == 0 || !splitFractionParameters.calculateSplitFractionsFromPositions())
      {
         return;
      }

      double defaultTransferSplitFraction = splitFractionParameters.getDefaultTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      for (int stepNumber = 0; stepNumber < numberOfStepsProvider.getAsInt(); stepNumber++)
      {
         if (stepNumber == 0)
         {
            RobotSide initialStanceSide = stepGetter.apply(stepNumber).getRobotSide().getOppositeSide();
            stanceFootPose.setToZero(soleFrames.get(initialStanceSide));
            stanceFootPose.changeFrame(worldFrame);
         }
         else
         {
            stanceFootPose.set(stepGetter.apply(stepNumber - 1).getFootstepPose());
         }

         nextFootPose.set(stepGetter.apply(stepNumber).getFootstepPose());

         // This step is a big step down.
         double stepDownHeight = nextFootPose.getZ() - stanceFootPose.getZ();

         if (stepDownHeight < -splitFractionParameters.getStepHeightForLargeStepDown())
         {
            double alpha = Math.min(1.0,
                                    (Math.abs(stepDownHeight) - splitFractionParameters.getStepHeightForLargeStepDown()) / (
                                          splitFractionParameters.getLargestStepDownHeight() - splitFractionParameters.getStepHeightForLargeStepDown()));
            double transferSplitFraction = InterpolationTools.linearInterpolate(defaultTransferSplitFraction,
                                                                                splitFractionParameters.getTransferSplitFractionAtFullDepth(), alpha);


            if (stepNumber == numberOfStepsProvider.getAsInt() - 1)
            { // this is the last step
               double currentSplitFraction = finalTransferSplitFractionProvider.getAsDouble();
               double currentWeightDistribution = finalTransferWeightDistributionProvider.getAsDouble();

               double transferWeightDistribution = InterpolationTools.linearInterpolate(defaultWeightDistribution,
                                                                                        splitFractionParameters.getTransferFinalWeightDistributionAtFullDepth(), alpha);

               double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
               double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution, currentWeightDistribution, defaultWeightDistribution);

               finalTransferSplitFractionConsumer.accept(splitFractionToSet);
               finalTransferWeightDistributionConsumer.accept(weightDistributionToSet);
            }
            else
            {
               double currentSplitFraction = transferSplitFractionProvider.applyAsDouble(stepNumber + 1);
               double currentWeightDistribution = transferWeightDistributionProvider.applyAsDouble(stepNumber + 1);

               double transferWeightDistribution = InterpolationTools.linearInterpolate(defaultWeightDistribution,
                                                                                        splitFractionParameters.getTransferWeightDistributionAtFullDepth(), alpha);

               double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
               double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution, currentWeightDistribution, defaultWeightDistribution);

               transferSplitFractionConsumer.apply(stepNumber + 1).accept(splitFractionToSet);
               transferWeightDistributionConsumer.apply(stepNumber + 1).accept(weightDistributionToSet);
            }
         }
      }
   }
}
