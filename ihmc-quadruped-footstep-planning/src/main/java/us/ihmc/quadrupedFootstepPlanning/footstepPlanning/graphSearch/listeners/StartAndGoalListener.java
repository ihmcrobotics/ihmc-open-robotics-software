package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;

public interface StartAndGoalListener
{
   void setInitialPose(FramePose2DReadOnly startPose);
   void setGoalPose(FramePose2DReadOnly goalPose);
}
