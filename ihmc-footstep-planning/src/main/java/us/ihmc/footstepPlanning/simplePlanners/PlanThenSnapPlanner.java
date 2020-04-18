package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanThenSnapPlanner
{
   private final TurnWalkTurnPlanner turnWalkTurnPlanner;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private PlanarRegionsList planarRegionsList;
   private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;

   public PlanThenSnapPlanner(FootstepPlannerParametersBasics footstepPlannerParameters, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.turnWalkTurnPlanner = new TurnWalkTurnPlanner(footstepPlannerParameters);
      this.footPolygons = footPolygons;
      SnapAndWiggleSingleStepParameters parameters = new SnapAndWiggleSingleStepParameters();
      parameters.setWiggleInWrongDirectionThreshold(Double.NaN);
      snapAndWiggleSingleStep = new SnapAndWiggleSingleStep(parameters);
   }

   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide stanceSide)
   {
      turnWalkTurnPlanner.setInitialStanceFoot(stanceFootPose, stanceSide);
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      turnWalkTurnPlanner.setGoal(goal);
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   private FootstepPlan footstepPlan = new FootstepPlan();

   public FootstepPlanningResult plan() throws SnappingFailedException
   {
      FootstepPlanningResult result = turnWalkTurnPlanner.plan();
      footstepPlan = turnWalkTurnPlanner.getPlan();

      if (planarRegionsList == null)
         return result;

      snapAndWiggleSingleStep.setPlanarRegions(planarRegionsList);

      int numberOfFootsteps = footstepPlan.getNumberOfSteps();
      for (int i = 0; i < numberOfFootsteps; i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);
         FramePose3D solePose = new FramePose3D();
         footstep.getSoleFramePose(solePose);
         ConvexPolygon2D footHold = snapAndWiggleSingleStep.snapAndWiggle(solePose, footPolygons.get(footstep.getRobotSide()), true);
         footstep.setSoleFramePose(solePose);
         if (footHold != null)
         {
            footstep.setFoothold(footHold);
         }
      }
      return result;
   }

   public FootstepPlan getPlan()
   {
      return footstepPlan;
   }
}
