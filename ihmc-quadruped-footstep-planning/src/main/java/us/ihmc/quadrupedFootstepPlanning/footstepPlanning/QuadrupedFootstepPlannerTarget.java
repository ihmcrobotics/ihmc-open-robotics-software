package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFootstepPlannerTarget
{
   protected final QuadrantDependentList<FramePoint3D> feetTargetPositions = new QuadrantDependentList<>(FramePoint3D::new);
   protected final FramePose3D targetPose = new FramePose3D();
   protected FootstepPlannerTargetType targetType = FootstepPlannerTargetType.POSE_BETWEEN_FEET;

   public FramePose3DReadOnly getTargetPose()
   {
      return targetPose;
   }

   public FramePoint3DReadOnly getFootGoalPosition(RobotQuadrant robotQuadrant)
   {
      return feetTargetPositions.get(robotQuadrant);
   }

   public QuadrantDependentList<? extends FramePoint3DReadOnly> getFeetTargetPositions()
   {
      return feetTargetPositions;
   }

   public FootstepPlannerTargetType getTargetType()
   {
      return targetType;
   }
}
