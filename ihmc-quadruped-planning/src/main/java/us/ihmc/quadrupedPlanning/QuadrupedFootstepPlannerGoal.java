package us.ihmc.quadrupedPlanning;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

public class QuadrupedFootstepPlannerGoal
{
   private FramePose3D goalPose;

   public void setGoalPose(FramePose3D goalPose)
   {
      this.goalPose = goalPose;
   }

   public FramePose3DReadOnly getGoalPose()
   {
      return goalPose;
   }
}
