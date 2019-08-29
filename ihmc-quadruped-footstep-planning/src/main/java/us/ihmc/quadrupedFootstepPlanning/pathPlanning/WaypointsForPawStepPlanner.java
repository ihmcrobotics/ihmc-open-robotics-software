package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerGoal;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public interface WaypointsForPawStepPlanner
{
   void setInitialBodyPose(FramePose3DReadOnly bodyPose);

   void setGoal(PawStepPlannerGoal goal);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   List<Point3D> getWaypoints();

   PawStepPlanningResult planWaypoints();

   FramePose3DReadOnly getInitialBodyPose();
   FramePose3DReadOnly getGoalBodyPose();

   void setFallbackRegionSize(double size);
}
