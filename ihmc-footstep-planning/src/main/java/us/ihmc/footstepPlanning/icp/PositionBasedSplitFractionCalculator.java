package us.ihmc.footstepPlanning.icp;

import javafx.scene.transform.Scale;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.SplitFractionFromPositionCalculator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.SplitFractionTools;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * The purpose of this class is to modify elements of the dynamic trajectory planner based on the step position.
 * Currently, the transfer phase is split into two segments. One where the CoP goes from the trailing foot to some midpoint, then the second where it goes
 * from the midpoint to the leading foot. The midpoint is determined by interpolating between the trailing foot and the leading foot by some fraction set by
 * the desired weight distribution {@link PlannedFootstep#setTransferWeightDistribution}. The time spent shifting from the trailing foot to the midpoint
 * is some fraction of the transfer duration determined by the transfer split fraction {@link PlannedFootstep#setTransferSplitFraction}.
 *
 * <p>
 * This module defines several parameters that allow the weight distribution and split fraction to be modified based on the step position, specifically
 * changes in step height. This is done so that the robot "commits" to large step downs. That is to say, when the robot is stepping down, there is a minimum
 * step down height that causes it to start to shift its upcoming CoP midpoint and time to cause it to rapidly shift the weight on the next step.
 * </p>
 * <p>
 * This is done by defining the desired weight distribution and split fraction if the upcoming foothold is below a certain threshold, defined by some maximum
 * step down distance. Then, once the step is below another threshold, it starts linearly interpolating to these values, until it has reached its largest
 * step down distance.
 * </p>
 */
public class PositionBasedSplitFractionCalculator
{
   private final SplitFractionCalculatorParametersReadOnly parameters;
   private final SplitFractionFromPositionCalculator calculator;

   public PositionBasedSplitFractionCalculator(SplitFractionCalculatorParametersReadOnly parameters)
   {
      this.parameters = parameters;
      this.calculator = new SplitFractionFromPositionCalculator(parameters);
   }

   public void computeSplitFractions(FootstepPlannerRequest request, FootstepPlan footstepPlan)
   {
      computeSplitFractions(footstepPlan, request.getStartFootPoses());
   }

   public void computeSplitFractions(FootstepPlan footstepPlan, SideDependentList<? extends Pose3DReadOnly> startFootPoses)
   {
      calculator.setNumberOfStepsProvider(footstepPlan::getNumberOfSteps);

      calculator.setFinalTransferSplitFractionProvider(footstepPlan::getFinalTransferSplitFraction);
      calculator.setFinalTransferWeightDistributionProvider(footstepPlan::getFinalTransferWeightDistribution);

      calculator.setTransferSplitFractionProvider((i) -> footstepPlan.getFootstep(i).getTransferSplitFraction());
      calculator.setTransferWeightDistributionProvider((i) -> footstepPlan.getFootstep(i).getTransferWeightDistribution());

      calculator.setFinalTransferSplitFractionConsumer(footstepPlan::setFinalTransferSplitFraction);
      calculator.setFinalTransferWeightDistributionConsumer(footstepPlan::setFinalTransferWeightDistribution);

      calculator.setTransferWeightDistributionConsumer((i) -> (d) -> footstepPlan.getFootstep(i).setTransferWeightDistribution(d));
      calculator.setTransferSplitFractionConsumer((i) -> (d) -> footstepPlan.getFootstep(i).setTransferSplitFraction(d));

      calculator.setFirstSupportPoseProvider(() -> startFootPoses.get(footstepPlan.getFootstep(0).getRobotSide()));
      calculator.setStepPoseGetter(i -> footstepPlan.getFootstep(i).getFootstepPose());

      calculator.computeSplitFractionsFromPosition();

      /*
      if (footstepPlan.getNumberOfSteps() == 0)
      {
         return;
      }

      FramePose3D stanceFootPose = new FramePose3D();
      FramePose3D nextFootPose = new FramePose3D();

      double defaultTransferSplitFraction = parameters.getDefaultTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      for (int stepNumber = 0; stepNumber < footstepPlan.getNumberOfSteps(); stepNumber++)
      {
         if (stepNumber == 0)
         {
            Pose3DReadOnly initialStancePose = startFootPoses.get(footstepPlan.getFootstep(stepNumber).getRobotSide().getOppositeSide());
            stanceFootPose.set(initialStancePose);
         }
         else
         {
            stanceFootPose.set(footstepPlan.getFootstep(stepNumber - 1).getFootstepPose());
         }

         nextFootPose.set(footstepPlan.getFootstep(stepNumber).getFootstepPose());

         // This step is a big step down.
         double stepDownHeight = nextFootPose.getZ() - stanceFootPose.getZ();

         if (stepDownHeight < -parameters.getStepHeightForLargeStepDown())
         {
            double alpha = Math.min(1.0, (Math.abs(stepDownHeight) - parameters.getStepHeightForLargeStepDown()) / (parameters.getLargestStepDownHeight() - parameters.getStepHeightForLargeStepDown()));
            double transferSplitFraction = InterpolationTools.linearInterpolate(defaultTransferSplitFraction,
                                                                                parameters.getTransferSplitFractionAtFullDepth(), alpha);
            double transferWeightDistribution = InterpolationTools.linearInterpolate(defaultWeightDistribution,
                                                                                     parameters.getTransferWeightDistributionAtFullDepth(), alpha);

            if (stepNumber == footstepPlan.getNumberOfSteps() - 1)
            { // this is the last step
               double currentSplitFraction = footstepPlan.getFinalTransferSplitFraction();
               double currentWeightDistribution = footstepPlan.getFinalTransferWeightDistribution();

               double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
               double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution, currentWeightDistribution, defaultWeightDistribution);

               footstepPlan.setFinalTransferSplitFraction(splitFractionToSet);
               footstepPlan.setFinalTransferWeightDistribution(weightDistributionToSet);
            }
            else
            {
               PlannedFootstep nextFootstep = footstepPlan.getFootstep(stepNumber + 1);

               double currentSplitFraction = nextFootstep.getTransferSplitFraction();
               double currentWeightDistribution = nextFootstep.getTransferWeightDistribution();

               double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
               double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution, currentWeightDistribution, defaultWeightDistribution);

               nextFootstep.setTransferSplitFraction(splitFractionToSet);
               nextFootstep.setTransferWeightDistribution(weightDistributionToSet);
            }
         }
      }

       */
   }
}
