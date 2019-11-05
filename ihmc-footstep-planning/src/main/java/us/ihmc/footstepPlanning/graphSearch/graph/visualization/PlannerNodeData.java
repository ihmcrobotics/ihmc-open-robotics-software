package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class PlannerNodeData
{
   private final int parentNodeId;
   private final RobotSide robotSide;

   private final Pose3D pose = new Pose3D();

   private final BipedalFootstepPlannerNodeRejectionReason rejectionReason;

   public PlannerNodeData(int parentNodeId, RobotSide robotSide, Pose3DReadOnly pose, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      this.parentNodeId = parentNodeId;
      this.robotSide = robotSide;
      this.pose.set(pose);
      this.rejectionReason = rejectionReason;
   }

   public int getParentNodeId()
   {
      return parentNodeId;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public Pose3DReadOnly getNodePose()
   {
      return pose;
   }

   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason;
   }
}
