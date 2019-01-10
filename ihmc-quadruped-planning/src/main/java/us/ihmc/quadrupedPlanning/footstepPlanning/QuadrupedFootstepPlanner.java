package us.ihmc.quadrupedPlanning.footstepPlanning;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public interface QuadrupedFootstepPlanner
{
   void setGroundPlane(QuadrupedGroundPlaneMessage groundPlane);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   void setInitialBodyPose(FramePose3DReadOnly bodyPose);

   void setGoal(QuadrupedFootstepPlannerGoal goal);

   void initialize();

   void plan();

   void update();

   List<? extends QuadrupedTimedStep> getSteps();
}
