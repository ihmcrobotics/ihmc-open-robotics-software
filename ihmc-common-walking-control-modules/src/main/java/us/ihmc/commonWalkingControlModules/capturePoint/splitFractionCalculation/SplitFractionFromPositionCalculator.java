package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.tools.functional.IntDoubleConsumer;

import java.util.function.*;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class SplitFractionFromPositionCalculator
{
   private final FramePose3D stanceFootPose = new FramePose3D();
   private final FramePose3D nextFootPose = new FramePose3D();

   private final SplitFractionCalculatorParametersReadOnly splitFractionParameters;

   private IntSupplier numberOfStepsProvider;
   private IntFunction<FramePose3DReadOnly> stepPoseGetter;
   private Supplier<? extends Pose3DReadOnly> firstSupportPoseProvider;
   private Supplier<? extends Pose3DReadOnly> firstSwingPoseProvider;
   private DoubleSupplier finalTransferWeightDistributionProvider;
   private DoubleSupplier finalTransferSplitFractionProvider;
   private IntToDoubleFunction transferWeightDistributionProvider;
   private IntToDoubleFunction transferSplitFractionProvider;
   private DoubleConsumer finalTransferWeightDistributionConsumer;
   private DoubleConsumer finalTransferSplitFractionConsumer;
   private IntDoubleConsumer transferWeightDistributionConsumer;
   private IntDoubleConsumer transferSplitFractionConsumer;

   public SplitFractionFromPositionCalculator(SplitFractionCalculatorParametersReadOnly splitFractionParameters)
   {
      this.splitFractionParameters = splitFractionParameters;
   }

   public void setNumberOfStepsProvider(IntSupplier numberOfStepsProvider)
   {
      this.numberOfStepsProvider = numberOfStepsProvider;
   }

   public void setFinalTransferWeightDistributionProvider(DoubleSupplier finalTransferWeightDistributionProvider)
   {
      this.finalTransferWeightDistributionProvider = finalTransferWeightDistributionProvider;
   }

   public void setFinalTransferSplitFractionProvider(DoubleSupplier finalTransferSplitFractionProvider)
   {
      this.finalTransferSplitFractionProvider = finalTransferSplitFractionProvider;
   }

   public void setFinalTransferWeightDistributionConsumer(DoubleConsumer finalTransferWeightDistributionConsumer)
   {
      this.finalTransferWeightDistributionConsumer = finalTransferWeightDistributionConsumer;
   }

   public void setFinalTransferSplitFractionConsumer(DoubleConsumer finalTransferSplitFractionConsumer)
   {
      this.finalTransferSplitFractionConsumer = finalTransferSplitFractionConsumer;
   }

   public void setTransferWeightDistributionProvider(IntToDoubleFunction transferWeightDistributionProvider)
   {
      this.transferWeightDistributionProvider = transferWeightDistributionProvider;
   }

   public void setTransferSplitFractionProvider(IntToDoubleFunction transferSplitFractionProvider)
   {
      this.transferSplitFractionProvider = transferSplitFractionProvider;
   }

   public void setTransferWeightDistributionConsumer(IntDoubleConsumer transferWeightDistributionConsumer)
   {
      this.transferWeightDistributionConsumer = transferWeightDistributionConsumer;
   }

   public void setTransferSplitFractionConsumer(IntDoubleConsumer transferSplitFractionConsumer)
   {
      this.transferSplitFractionConsumer = transferSplitFractionConsumer;
   }

   public void setFirstSupportPoseProvider(Supplier<? extends Pose3DReadOnly> firstSupportPoseProvider)
   {
      this.firstSupportPoseProvider = firstSupportPoseProvider;
   }

   public void setFirstSwingPoseProvider(Supplier<? extends Pose3DReadOnly> firstSwingPoseProvider)
   {
      this.firstSwingPoseProvider = firstSwingPoseProvider;
   }

   public void setStepPoseGetter(IntFunction<FramePose3DReadOnly> stepPoseGetter)
   {
      this.stepPoseGetter = stepPoseGetter;
   }

   public void computeSplitFractionsFromPosition()
   {
      if (numberOfStepsProvider.getAsInt() == 0 || !splitFractionParameters.calculateSplitFractionsFromPositions())
      {
         return;
      }

      double defaultTransferSplitFraction = splitFractionParameters.getDefaultTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      // handle the initial transfer state
      {
         // transfer to foot
         stanceFootPose.setIncludingFrame(worldFrame, firstSupportPoseProvider.get());
         // upcoming swing foot
         nextFootPose.setIncludingFrame(worldFrame, firstSwingPoseProvider.get());

         double stepHeight = stanceFootPose.getZ() - nextFootPose.getZ();
         boolean isABigStepDown = stepHeight < -splitFractionParameters.getStepHeightForLargeStepDown();
         boolean isABigStepUp = stepHeight > splitFractionParameters.getStepHeightForLargeStepUp();
         if (isABigStepDown || isABigStepUp)
         {  // This step is either a big step up or a big step down. So do the calculation accordingly.

            // what constitutes a large step? This determines how far up or down to start shifting.
            double largeStepHeight = isABigStepDown ?
                  splitFractionParameters.getStepHeightForLargeStepDown() :
                  splitFractionParameters.getStepHeightForLargeStepUp();
            // This determines how far up or down to saturate.
            double largestStepHeight = isABigStepDown ? splitFractionParameters.getLargestStepDownHeight() : splitFractionParameters.getLargestStepUpHeight();

            // This is like an alpha value for how "down" or "up" the step is, and must be between 0 and 1.
            double alpha = MathTools.clamp((Math.abs(stepHeight) - largeStepHeight) / (largestStepHeight - largeStepHeight), 0.0, 1.0);

            // This says what the extreme value of the split fraction should be if stepping or down. If down, we likely want to initially move the CoP very
            // quickly, and then move it slowly (like loading the foot quickly). If up, we want the opposite.
            double splitFractionAtFullDepth = isABigStepDown ?
                  splitFractionParameters.getTransferSplitFractionAtFullDepth() :
                  splitFractionParameters.getTransferSplitFractionForStepUpAtFullDepth();

            // Figure out what the actual split fraction should be, based on how far up or down they are.
            double transferSplitFraction = InterpolationTools.linearInterpolate(defaultTransferSplitFraction, splitFractionAtFullDepth, alpha);

            // This says what the extreme value of the weight distribution should be if stepping or down. If down, we likely want to the CoP to be closer to the
            // upcoming foot. If up, we want the opposite.
            double splitFractionWeightDistributionAtFullDepth = isABigStepDown ?
                  splitFractionParameters.getTransferWeightDistributionAtFullDepth() :
                  splitFractionParameters.getTransferWeightDistributionForStepUpAtFullDepth();
            // Figure out what the actual weight distribution should be, based on how far up or down they are.
            double transferWeightDistribution = InterpolationTools.linearInterpolate(defaultWeightDistribution,
                                                                                     splitFractionWeightDistributionAtFullDepth,
                                                                                     alpha);

            // Apply the split fraction and weight distribution to the base value set forth by the planner.
            double currentSplitFraction = transferSplitFractionProvider.applyAsDouble(0);
            double currentWeightDistribution = transferWeightDistributionProvider.applyAsDouble(0);

            double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
            double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution,
                                                                                         currentWeightDistribution,
                                                                                         defaultWeightDistribution);

            transferSplitFractionConsumer.accept(0, splitFractionToSet);
            transferWeightDistributionConsumer.accept(0, weightDistributionToSet);
         }
      }

      for (int stepNumber = 0; stepNumber < numberOfStepsProvider.getAsInt(); stepNumber++)
      {
         if (stepNumber == 0)
         {
            stanceFootPose.setIncludingFrame(worldFrame, firstSupportPoseProvider.get());
         }
         else
         {
            stanceFootPose.set(stepPoseGetter.apply(stepNumber - 1));
         }

         nextFootPose.set(stepPoseGetter.apply(stepNumber));

         double stepHeight = nextFootPose.getZ() - stanceFootPose.getZ();
         boolean isABigStepDown = stepHeight < -splitFractionParameters.getStepHeightForLargeStepDown();
         boolean isABigStepUp = stepHeight > splitFractionParameters.getStepHeightForLargeStepUp();
         if (isABigStepDown || isABigStepUp)
         {  // This step is either a big step up or a big step down.
            double largeStepHeight = isABigStepDown ?
                  splitFractionParameters.getStepHeightForLargeStepDown() :
                  splitFractionParameters.getStepHeightForLargeStepUp();
            double largestStepHeight = isABigStepDown ? splitFractionParameters.getLargestStepDownHeight() : splitFractionParameters.getLargestStepUpHeight();

            double alpha = Math.min(1.0, (Math.abs(stepHeight) - largeStepHeight) / (largestStepHeight - largeStepHeight));

            double splitFractionAtFullDepth = isABigStepDown ?
                  splitFractionParameters.getTransferSplitFractionAtFullDepth() :
                  splitFractionParameters.getTransferSplitFractionForStepUpAtFullDepth();

            double transferSplitFraction = InterpolationTools.linearInterpolate(defaultTransferSplitFraction, splitFractionAtFullDepth, alpha);

            if (stepNumber == numberOfStepsProvider.getAsInt() - 1)
            { // this is the last step
               double currentSplitFraction = finalTransferSplitFractionProvider.getAsDouble();
               double currentWeightDistribution = finalTransferWeightDistributionProvider.getAsDouble();

               double transferFinalWeightDistribution = isABigStepDown ?
                     splitFractionParameters.getTransferFinalWeightDistributionAtFullDepth() :
                     splitFractionParameters.getTransferFinalWeightDistributionForStepUpAtFullDepth();
               double transferWeightDistribution = InterpolationTools.linearInterpolate(defaultWeightDistribution, transferFinalWeightDistribution, alpha);

               double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
               double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution,
                                                                                            currentWeightDistribution,
                                                                                            defaultWeightDistribution);

               finalTransferSplitFractionConsumer.accept(splitFractionToSet);
               finalTransferWeightDistributionConsumer.accept(weightDistributionToSet);
            }
            else
            {
               double currentSplitFraction = transferSplitFractionProvider.applyAsDouble(stepNumber + 1);
               double currentWeightDistribution = transferWeightDistributionProvider.applyAsDouble(stepNumber + 1);

               double weightDistributionAtFullDepth = isABigStepDown ?
                     splitFractionParameters.getTransferWeightDistributionAtFullDepth() :
                     splitFractionParameters.getTransferWeightDistributionForStepUpAtFullDepth();
               double transferWeightDistribution = InterpolationTools.linearInterpolate(defaultWeightDistribution, weightDistributionAtFullDepth, alpha);

               // Apply the split fraction and weight distribution to the base value set forth by the planner.
               double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
               double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution,
                                                                                            currentWeightDistribution,
                                                                                            defaultWeightDistribution);

               transferSplitFractionConsumer.accept(stepNumber + 1, splitFractionToSet);
               transferWeightDistributionConsumer.accept(stepNumber + 1, weightDistributionToSet);
            }
         }
      }
   }
}
