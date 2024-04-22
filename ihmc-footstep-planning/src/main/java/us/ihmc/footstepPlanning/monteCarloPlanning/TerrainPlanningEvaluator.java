package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.SideDependentList;

public class TerrainPlanningEvaluator
{
   public double computeTotalReward(FootstepPlan plan, TerrainMapData terrainMap, SideDependentList<FramePose3D> goalStancePoses)
   {
      double totalReward = 0.0;
      for (int i = 0; i<plan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = plan.getFootstep(i);

         double distanceFromLastStepToGoal = footstep.getFootstepPose().getPosition().distance(goalStancePoses.get(footstep.getRobotSide()).getPosition());
         double distanceFromLastOppositeStepToGoal = footstep.getFootstepPose().getPosition().distance(goalStancePoses.get(footstep.getRobotSide().getOppositeSide()).getPosition());

         totalReward += MathTools.clamp(1.0f / distanceFromLastOppositeStepToGoal + 1.0f / distanceFromLastStepToGoal, 0.0, 20) * 10;

         double contactScore = terrainMap.getContactScoreInWorld((float) footstep.getFootstepPose().getPosition().getX(),
                                                                 (float) footstep.getFootstepPose().getPosition().getY());

         totalReward += contactScore;

      }
      return totalReward;
   }
}
