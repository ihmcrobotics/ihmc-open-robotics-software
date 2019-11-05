package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
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

   public PlannerNodeData(FootstepNodeDataMessage message)
   {
      this(message.getParentNodeId(), RobotSide.fromByte(message.getRobotSide()), new Pose3D(message.getPosition(), message.getOrientation()),
           BipedalFootstepPlannerNodeRejectionReason.fromByte(message.getBipedalFootstepPlannerNodeRejectionReason()));
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

   public FootstepNodeDataMessage getAsMessage()
   {
      FootstepNodeDataMessage message = new FootstepNodeDataMessage();
      getAsMessage(message);
      return message;
   }

   public void getAsMessage(FootstepNodeDataMessage message)
   {
      byte rejectionReason = getRejectionReason() != null ? getRejectionReason().toByte() : (byte) 255;

      message.setParentNodeId(getParentNodeId());
      message.setRobotSide(getRobotSide().toByte());
      message.getPosition().set(getNodePose().getPosition());
      message.getOrientation().set(getNodePose().getOrientation());
      message.setBipedalFootstepPlannerNodeRejectionReason(rejectionReason);
   }
}
