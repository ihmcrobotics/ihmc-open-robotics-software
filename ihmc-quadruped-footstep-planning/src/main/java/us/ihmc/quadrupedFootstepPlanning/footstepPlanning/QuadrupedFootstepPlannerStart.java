package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFootstepPlannerStart extends QuadrupedFootstepPlannerTarget
{
   public void setStartPose(FramePose3DReadOnly startPose)
   {
      setStartType(FootstepPlannerTargetType.POSE_BETWEEN_FEET);
      this.targetPose.set(startPose);
   }

   public void setFootStartPosition(RobotQuadrant robotQuadrant, FramePoint3DReadOnly startPosition)
   {
      setStartType(FootstepPlannerTargetType.FOOTSTEPS);
      this.feetTargetPositions.get(robotQuadrant).set(startPosition);
   }

   public void setStartType(FootstepPlannerTargetType startType)
   {
      this.targetType = startType;
   }
}
