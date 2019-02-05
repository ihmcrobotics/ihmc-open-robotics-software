package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFootstepPlannerGoal extends QuadrupedFootstepPlannerTarget
{
   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      setGoalType(FootstepPlannerTargetType.POSE_BETWEEN_FEET);
      this.targetPose.set(goalPose);
   }

   public void setFootGoalPosition(RobotQuadrant robotQuadrant, FramePoint3DReadOnly goalPosition)
   {
      setGoalType(FootstepPlannerTargetType.FOOTSTEPS);
      this.feetTargetPositions.get(robotQuadrant).set(goalPosition);
   }

   public void setGoalType(FootstepPlannerTargetType goalType)
   {
      this.targetType = goalType;
   }
}
