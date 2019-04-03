package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanThenSnapPlanner implements BodyPathAndFootstepPlanner
{
   private final FootstepPlanner internalPlanner;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private PlanarRegionsList planarRegionsList;
   private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;

   public PlanThenSnapPlanner(FootstepPlanner internalPlanner, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.internalPlanner = internalPlanner;
      this.footPolygons = footPolygons;
      SnapAndWiggleSingleStepParameters parameters = new SnapAndWiggleSingleStepParameters();
      parameters.setWiggleInWrongDirectionThreshold(Double.NaN);
      snapAndWiggleSingleStep = new SnapAndWiggleSingleStep(parameters);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide stanceSide)
   {
      internalPlanner.setInitialStanceFoot(stanceFootPose, stanceSide);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      internalPlanner.setGoal(goal);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   private FootstepPlan footstepPlan = new FootstepPlan();

   @Override
   public FootstepPlanningResult plan()
   {
      FootstepPlanningResult result = internalPlanner.plan();
      footstepPlan = internalPlanner.getPlan();

      if (planarRegionsList == null)
         return result;

      snapAndWiggleSingleStep.setPlanarRegions(planarRegionsList);

      int numberOfFootsteps = footstepPlan.getNumberOfSteps();
      for (int i=0; i<numberOfFootsteps; i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);
         try
         {
            FramePose3D solePose = new FramePose3D();
            footstep.getSoleFramePose(solePose);
            ConvexPolygon2D footHold = snapAndWiggleSingleStep.snapAndWiggle(solePose, footPolygons.get(footstep.getRobotSide()), true);
            footstep.setSoleFramePose(solePose);
            if(footHold!=null)
            {
               footstep.setFoothold(footHold);
            }
         }
         catch (Exception e)
         {
            return FootstepPlanningResult.SNAPPING_FAILED;
         }
      }
      return result;
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlan;
   }

   @Override
   public void setTimeout(double timeout)
   {

   }

   @Override
   public double getPlanningDuration()
   {
      return -1;
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizonLength)
   {
   }

}
