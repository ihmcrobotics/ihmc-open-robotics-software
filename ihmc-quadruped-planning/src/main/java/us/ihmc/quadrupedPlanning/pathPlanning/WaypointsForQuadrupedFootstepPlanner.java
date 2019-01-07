package us.ihmc.quadrupedPlanning.pathPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public interface WaypointsForQuadrupedFootstepPlanner
{
   void setInitialBodyPose(FramePose3DReadOnly bodyPose);

   void setGoal(QuadrupedFootstepPlannerGoal goal);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   List<Point3D> getWaypoints();

   void planWaypoints();
}
