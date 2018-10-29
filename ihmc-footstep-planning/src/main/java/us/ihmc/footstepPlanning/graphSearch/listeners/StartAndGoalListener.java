package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.referenceFrame.FramePose3D;

public interface StartAndGoalListener
{
   void setStartPose(FramePose3D startPose);
   void setGoalPose(FramePose3D goalPose);
}
