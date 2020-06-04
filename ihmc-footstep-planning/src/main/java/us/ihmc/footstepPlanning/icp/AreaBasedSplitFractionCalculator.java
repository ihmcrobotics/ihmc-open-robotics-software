package us.ihmc.footstepPlanning.icp;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * The purpose of this class is to modify elements of the dynamic trajectory planner based on the comparative quality between footholds.
 * Currently, the transfer phase is split into two segments. One where the CoP goes from the trailing foot to some midpoint, then the second where it goes
 * from the midpoint to the leading foot. The midpoint is determined by interpolating between the trailing foot and the leading foot by some fraction set by
 * the desired weight distribution {@link PlannedFootstep#setTransferWeightDistribution}}. The time spent shifting from the trailing foot to the midpoint
 * is some fraction of the transfer duration determined by the transfer split fraction {@link PlannedFootstep#setTransferSplitFraction}.
 *
 * <p>
 *    This module defines several parameters that allow the weight distribution and split fraction to be modified based on the comparative area and width of the
 *    leading and trailing footholds. That is, we can say that if the upcoming foothold has more area, we should shift our weight to be more on it, and do so
 *    quickly. The same is said for the width - if it has more width, we should shift out weight more and quickly.
 * </p>
 * <p>
 *    This is done by defining the desired weight distribution and split fraction if the upcoming foothold has ALL the area and width. That is, if the trailing
 *    foot has no support area, where the midpoint CoP should be located, and if the trailing foot is purely a line, where the midpoint CoP should be.
 * </p>
 */
public class AreaBasedSplitFractionCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SplitFractionCalculatorParametersReadOnly parameters;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   public AreaBasedSplitFractionCalculator(SplitFractionCalculatorParametersReadOnly parameters,
                                           SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
   }

   public void computeSplitFractions(FootstepPlannerRequest request, FootstepPlan footstepPlan)
   {
      if (footstepPlan.getNumberOfSteps() == 0)
      {
         return;
      }

      ConvexPolygon2D previousPolygon = new ConvexPolygon2D();
      ConvexPolygon2D currentPolygon = new ConvexPolygon2D();

      PoseReferenceFrame previousFrame = new PoseReferenceFrame("previousFrame", worldFrame);
      PoseReferenceFrame currentFrame = new PoseReferenceFrame("nextFrame", worldFrame);

      double defaultTransferSplitFraction = parameters.getDefaultTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      for (int stepNumber = 0; stepNumber < footstepPlan.getNumberOfSteps(); stepNumber++)
      {
         if (stepNumber == 0)
         {
            RobotSide stanceSide = footstepPlan.getFootstep(0).getRobotSide().getOppositeSide();
            previousFrame.setPoseAndUpdate(request.getStartFootPoses().get(stanceSide));

            ConvexPolygon2D initialStanceFoothold = request.getStartFootholds().get(stanceSide);
            if (initialStanceFoothold.isEmpty())
            {
               previousPolygon.set(footPolygons.get(stanceSide));
            }
            else
            {
               previousPolygon.set(initialStanceFoothold);
            }
         }
         else
         {
            PlannedFootstep previousStep = footstepPlan.getFootstep(stepNumber - 1);
            previousFrame.setPoseAndUpdate(previousStep.getFootstepPose());
            previousPolygon.set(previousStep.getFoothold());
         }

         PlannedFootstep currentStep = footstepPlan.getFootstep(stepNumber);
         currentFrame.setPoseAndUpdate(currentStep.getFootstepPose());
         if (currentStep.hasFoothold())
         {
            currentPolygon.set(currentStep.getFoothold());
         }
         else
         {
            currentPolygon.set(footPolygons.get(currentStep.getRobotSide()));
         }

         double currentArea = currentPolygon.getArea();
         double previousArea = previousPolygon.getArea();

         double totalArea = currentArea + previousArea;

         double currentWidth = currentPolygon.getBoundingBoxRangeY();
         double previousWidth = previousPolygon.getBoundingBoxRangeY();

         double totalWidth = currentWidth + previousWidth;

         double percentAreaOnCurrentFoot = totalArea > 0.0 ? currentArea / totalArea : 0.5;
         double percentWidthOnCurrentFoot = totalWidth > 0.0 ? currentWidth / totalWidth : 0.5;

         if (MathTools.epsilonEquals(percentAreaOnCurrentFoot, 0.5, 1.0e-2) && MathTools.epsilonEquals(percentWidthOnCurrentFoot, 0.5, 2.0e-2))
            continue;

         double transferWeightDistributionFromArea = InterpolationTools.linearInterpolate(defaultWeightDistribution, parameters.getFractionLoadIfFootHasFullSupport(),
                                                                                          2.0 * percentAreaOnCurrentFoot - 1.0);
         double transferWeightDistributionFromWidth = InterpolationTools.linearInterpolate(defaultWeightDistribution, parameters.getFractionLoadIfOtherFootHasNoWidth(),
                                                                                           2.0 * percentWidthOnCurrentFoot - 1.0);

         // lower means it spends more time shifting to the center, higher means it spends less time shifting to the center
         // e.g., if we set the fraction to 0 and the trailing foot has no area, the split fraction should be 1 because we spend no time on the first segment
         double transferSplitFractionFromArea = InterpolationTools.linearInterpolate(defaultTransferSplitFraction, 1.0 - parameters.getFractionTimeOnFootIfFootHasFullSupport(),
                                                                                     2.0 * percentAreaOnCurrentFoot - 1.0);
         double transferSplitFractionFromWidth = InterpolationTools.linearInterpolate(defaultTransferSplitFraction, 1.0 - parameters.getFractionTimeOnFootIfOtherFootHasNoWidth(),
                                                                                      2.0 * percentWidthOnCurrentFoot - 1.0);

         double transferWeightDistribution = 0.5 * (transferWeightDistributionFromArea + transferWeightDistributionFromWidth);
         double transferSplitFraction = 0.5 * (transferSplitFractionFromArea + transferSplitFractionFromWidth);

         transferWeightDistribution = MathTools.clamp(transferWeightDistribution, 0.01, 0.99);
         transferSplitFraction = MathTools.clamp(transferSplitFraction, 0.01, 0.99);

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
}
