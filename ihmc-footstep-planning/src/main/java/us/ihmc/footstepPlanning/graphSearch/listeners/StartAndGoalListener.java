package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.referenceFrame.FramePose3D;

public interface StartAndGoalListener
{
   void setInitialPose(FramePose3D startPose);
   void setGoalPose(FramePose3D goalPose);
}
