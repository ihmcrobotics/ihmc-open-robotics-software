package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.RobotSide;

public class PlannerNodeData
{
   private final int parentNodeId;

   private final RigidBodyTransform pose;
   private final FootstepNode footstepNode;

   private final int hashCode;

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason;

   public PlannerNodeData(int parentNodeId, FootstepNode node, RigidBodyTransform pose, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      this.footstepNode = node;
      this.parentNodeId = parentNodeId;
      this.pose = pose;
      this.rejectionReason = rejectionReason;

      hashCode = footstepNode.hashCode();
   }

   public PlannerNodeData(FootstepNodeDataMessage message)
   {
      footstepNode = new FootstepNode(message.getFootstepNode().getXIndex(), message.getFootstepNode().getYIndex(), message.getFootstepNode().getYawIndex(),
                                      RobotSide.fromByte(message.getFootstepNode().getRobotSide()));
      footstepNode.setNodeIndex(message.getFootstepNode().getNodeIndex());
      parentNodeId = message.getParentNodeId();
      pose = new RigidBodyTransform(message.getOrientation(), message.getPosition());
      rejectionReason = BipedalFootstepPlannerNodeRejectionReason.fromByte(message.getBipedalFootstepPlannerNodeRejectionReason());

      hashCode = footstepNode.hashCode();
   }

   public FootstepNode getFootstepNode()
   {
      return footstepNode;
   }

   public int getNodeId()
   {
      return footstepNode.getNodeIndex();
   }

   public int getParentNodeId()
   {
      return parentNodeId;
   }

   public RigidBodyTransform getNodePose()
   {
      return pose;
   }

   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason;
   }

   public void setRejectionReason(BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      this.rejectionReason = rejectionReason;
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

      message.getFootstepNode().setNodeIndex(getNodeId());
      message.setParentNodeId(getParentNodeId());
      message.getFootstepNode().setXIndex(getFootstepNode().getXIndex());
      message.getFootstepNode().setYIndex(getFootstepNode().getYIndex());
      message.getFootstepNode().setYawIndex(getFootstepNode().getYawIndex());
      message.getFootstepNode().setRobotSide(getFootstepNode().getRobotSide().toByte());
      message.getPosition().set(getNodePose().getTranslation());
      message.getOrientation().set(getNodePose().getRotation());
      message.setBipedalFootstepPlannerNodeRejectionReason(rejectionReason);
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   @Override
   public boolean equals(Object obj)
   {
      return getFootstepNode().equals(obj);
   }
}
