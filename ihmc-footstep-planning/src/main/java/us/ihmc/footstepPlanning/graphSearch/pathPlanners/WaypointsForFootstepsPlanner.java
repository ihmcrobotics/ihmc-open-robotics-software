package us.ihmc.footstepPlanning.graphSearch.pathPlanners;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerObjective;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public interface WaypointsForFootstepsPlanner
{
   default void setFootstepPlannerObjective(FootstepPlannerObjective objective)
   {
      if (objective.hasInitialStanceFootPose())
         setInitialStanceFoot(objective.getInitialStanceFootPose(), objective.getInitialStanceFootSide());
      if (objective.hasTimeout())
         setTimeout(objective.getTimeout());
      if (objective.hasGoal())
         setGoal(objective.getGoal());
   }

   void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side);

   void setGoal(FootstepPlannerGoal goal);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   void setTimeout(double timeout);

   void computeBestEffortPlan(double horizonLength);

   List<Pose3DReadOnly> getWaypoints();

   FootstepPlanningResult planWaypoints();

   PlannerStatistics<?> getPlannerStatistics();

   void cancelPlanning();
}
