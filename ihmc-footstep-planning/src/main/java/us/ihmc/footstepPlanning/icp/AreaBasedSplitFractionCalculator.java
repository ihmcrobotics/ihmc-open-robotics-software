package us.ihmc.footstepPlanning.icp;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.SplitFractionFromAreaCalculator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.PlannedFootstep;
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
 * This module defines several parameters that allow the weight distribution and split fraction to be modified based on the comparative area and width of the
 * leading and trailing footholds. That is, we can say that if the upcoming foothold has more area, we should shift our weight to be more on it, and do so
 * quickly. The same is said for the width - if it has more width, we should shift out weight more and quickly.
 * </p>
 * <p>
 * This is done by defining the desired weight distribution and split fraction if the upcoming foothold has ALL the area and width. That is, if the trailing
 * foot has no support area, where the midpoint CoP should be located, and if the trailing foot is purely a line, where the midpoint CoP should be.
 * </p>
 */
public class AreaBasedSplitFractionCalculator
{
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final SplitFractionFromAreaCalculator calculator;

   public AreaBasedSplitFractionCalculator(SplitFractionCalculatorParametersPropertyReadOnly parameters, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.footPolygons = footPolygons;

      calculator = new SplitFractionFromAreaCalculator(parameters, footPolygons);
   }

   public void computeSplitFractions(FootstepPlannerRequest request, FootstepPlan footstepPlan)
   {
      computeSplitFractions(footstepPlan, request.getStartFootPoses(), request.getStartFootholds());
   }

   public void computeSplitFractions(FootstepPlan footstepPlan,
                                     SideDependentList<? extends Pose3DReadOnly> startFootPoses,
                                     SideDependentList<ConvexPolygon2D> startFootholds)
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

      calculator.setFirstSupportPolygonProvider(() ->
                                                {
                                                   RobotSide stanceSide = footstepPlan.getFootstep(0).getRobotSide().getOppositeSide();
                                                   ConvexPolygon2D initialStanceFoothold = startFootholds.get(stanceSide);
                                                   if (initialStanceFoothold.isEmpty())
                                                   {
                                                      return footPolygons.get(stanceSide).getPolygonVerticesView();
                                                   }
                                                   else
                                                   {
                                                      return initialStanceFoothold.getPolygonVerticesView();
                                                   }
                                                });
      calculator.setStepSideProvider((i) -> footstepPlan.getFootstep(i).getRobotSide());
      calculator.setStepPolygonGetter((i) -> footstepPlan.getFootstep(i).getFoothold().getPolygonVerticesView());

      calculator.computeSplitFractionsFromArea();
   }
}
