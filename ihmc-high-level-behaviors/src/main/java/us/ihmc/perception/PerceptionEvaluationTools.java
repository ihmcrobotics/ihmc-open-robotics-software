package us.ihmc.perception;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PerceptionEvaluationTools
{
   public static FootstepDataListMessage planFootsteps(FootstepPlanningModule footstepPlanningModule, PlanarRegionsList planarRegions, Pose3D startPose,
                                                       Pose3D goalPose)
   {
      SideDependentList<Pose3D> squaredUpStartFootsteps = PlannerTools.createSquaredUpFootsteps(startPose, 0.2);
      SideDependentList<Pose3D> squaredUpGoalFootsteps = PlannerTools.createSquaredUpFootsteps(goalPose, 0.2);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setStartFootPoses(squaredUpStartFootsteps.get(RobotSide.LEFT), squaredUpStartFootsteps.get(RobotSide.RIGHT));
      request.setGoalFootPoses(squaredUpGoalFootsteps.get(RobotSide.LEFT), squaredUpGoalFootsteps.get(RobotSide.RIGHT));
      request.setGoalYawProximity(100.0);
      request.setGoalDistanceProximity(0.3);
      request.setTimeout(0.5f);
      request.setPerformAStarSearch(true);
      request.setSnapGoalSteps(true);
      request.setAbortIfGoalStepSnappingFails(true);

      FootstepPlannerOutput plannerOutput = footstepPlanningModule.handleRequest(request);

      if (plannerOutput != null)
      {
         FootstepPlanningResult footstepPlanningResult = plannerOutput.getFootstepPlanningResult();

         LogTools.info("Result: {}" + String.format(", Plan Iterations: %d, Plan Time: %.4f, Total Time: %.4f, Steps: %d",
                                                    plannerOutput.getPlannerTimings().getStepPlanningIterations(),
                                                    plannerOutput.getPlannerTimings().getTimePlanningStepsSeconds(),
                                                    plannerOutput.getPlannerTimings().getTotalElapsedSeconds(),
                                                    plannerOutput.getFootstepPlan().getNumberOfSteps()),
                       footstepPlanningResult);

         FootstepDataListMessage message = FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 0.6, 0.3);
         return message;
      }

      return null;
   }
}
