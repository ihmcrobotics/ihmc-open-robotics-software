package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

public interface StartAndGoalPawListener
{
   void setInitialPose(FramePose2DReadOnly startPose);
   void setGoalPose(FramePose3DReadOnly goalPose);
}
