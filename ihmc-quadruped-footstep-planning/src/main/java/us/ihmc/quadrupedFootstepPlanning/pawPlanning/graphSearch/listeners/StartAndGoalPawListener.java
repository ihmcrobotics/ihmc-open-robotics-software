package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;

public interface StartAndGoalPawListener
{
   void setInitialPose(FramePose2DReadOnly startPose);
   void setGoalPose(FramePose2DReadOnly goalPose);
}
