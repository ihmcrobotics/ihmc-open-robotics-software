package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawPlannerTarget
{
   protected final QuadrantDependentList<FramePoint3D> pawTargetPositions = new QuadrantDependentList<>(FramePoint3D::new);
   protected final FramePose3D targetPose = new FramePose3D();
   protected PawPlannerTargetType targetType = PawPlannerTargetType.POSE_BETWEEN_FEET;

   public FramePose3DReadOnly getTargetPose()
   {
      return targetPose;
   }

   public FramePoint3DReadOnly getPawGoalPosition(RobotQuadrant robotQuadrant)
   {
      return pawTargetPositions.get(robotQuadrant);
   }

   public QuadrantDependentList<? extends FramePoint3DReadOnly> getPawTargetPositions()
   {
      return pawTargetPositions;
   }

   public PawPlannerTargetType getTargetType()
   {
      return targetType;
   }
}
