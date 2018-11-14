package us.ihmc.footstepPlanning.graphSearch.pathPlanners;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public interface WaypointsForFootstepsPlanner
{
   void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side);

   void setGoal(FootstepPlannerGoal goal);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   void computeBestEffortPlan(double horizonLength);

   List<Point3D> getWaypoints();

   FootstepPlanningResult planWaypoints();

   PlannerStatistics<?> getPlannerStatistics();

   void cancelPlanning();
}
