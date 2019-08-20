package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawPlannerStart extends PawPlannerTarget
{
   private RobotQuadrant initialQuadrant = null;

   public void setStartPose(FramePose3DReadOnly startPose)
   {
      setStartType(PawPlannerTargetType.POSE_BETWEEN_FEET);
      this.targetPose.set(startPose);
   }

   public void setPawStartPosition(RobotQuadrant robotQuadrant, FramePoint3DReadOnly startPosition)
   {
      setStartType(PawPlannerTargetType.FOOTSTEPS);
      this.pawTargetPositions.get(robotQuadrant).set(startPosition);
   }

   public void setStartType(PawPlannerTargetType startType)
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
