package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanThenSnapPlanner implements FootstepPlanner
{
   private final WiggleParameters wiggleParameters = new WiggleParameters();

   private final FootstepPlanner internalPlanner;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private PlanarRegionsList planarRegionsList;
   private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;

   public PlanThenSnapPlanner(FootstepPlanner internalPlanner, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.internalPlanner = internalPlanner;
      this.footPolygons = footPolygons;
      snapAndWiggleSingleStep = new SnapAndWiggleSingleStep();
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
            ConvexPolygon2D footHold = snapAndWiggleSingleStep.snapAndWiggle(solePose, footPolygons.get(footstep.getRobotSide()));
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

}
