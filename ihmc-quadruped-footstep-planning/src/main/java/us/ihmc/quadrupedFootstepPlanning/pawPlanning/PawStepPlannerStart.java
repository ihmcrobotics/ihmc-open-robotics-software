package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawStepPlannerStart extends PawStepPlannerTarget
{
   private RobotQuadrant initialQuadrant = null;

   public void setStartPose(FramePose3DReadOnly startPose)
   {
      setStartType(PawStepPlannerTargetType.POSE_BETWEEN_FEET);
      this.targetPose.set(startPose);
   }

   public void setPawStartPosition(RobotQuadrant robotQuadrant, FramePoint3DReadOnly startPosition)
   {
      setStartType(PawStepPlannerTargetType.FOOTSTEPS);
      this.pawTargetPositions.get(robotQuadrant).set(startPosition);
   }

   public void setStartType(PawStepPlannerTargetType startType)
   {
      this.targetType = startType;
   }

   public void setInitialQuadrant(RobotQuadrant initialQuadrant)
   {
      this.initialQuadrant = initialQuadrant;
   }

   public RobotQuadrant getInitialQuadrant()
   {
      return initialQuadrant;
   }
}
