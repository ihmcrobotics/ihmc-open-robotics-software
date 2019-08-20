package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlannerGoal;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public interface WaypointsForPawPlanner
{
   void setInitialBodyPose(FramePose3DReadOnly bodyPose);

   void setGoal(PawPlannerGoal goal);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   List<Point3D> getWaypoints();

   PawPlanningResult planWaypoints();

   FramePose3DReadOnly getInitialBodyPose();
   FramePose3DReadOnly getGoalBodyPose();

   void setFallbackRegionSize(double size);
}
