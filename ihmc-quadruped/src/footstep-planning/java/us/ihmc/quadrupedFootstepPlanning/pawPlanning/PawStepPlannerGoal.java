package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawStepPlannerGoal extends PawStepPlannerTarget
{
   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      setGoalType(PawStepPlannerTargetType.POSE_BETWEEN_FEET);
      this.targetPose.set(goalPose);
   }

   public void setPawGoalPosition(RobotQuadrant robotQuadrant, FramePoint3DReadOnly goalPosition)
   {
      setGoalType(PawStepPlannerTargetType.FOOTSTEPS);
      this.pawTargetPositions.get(robotQuadrant).set(goalPosition);
   }

   public void setGoalType(PawStepPlannerTargetType goalType)
   {
      this.targetType = goalType;
   }
}
