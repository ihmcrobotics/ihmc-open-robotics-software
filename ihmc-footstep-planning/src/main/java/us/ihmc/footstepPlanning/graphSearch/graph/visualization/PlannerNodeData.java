package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.robotics.robotSide.RobotSide;

public class PlannerNodeData
{
   private final int parentNodeId;
   private final int nodeId;

   private final Pose3D pose = new Pose3D();
   private final FootstepNode footstepNode;

   private final int hashCode;

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason;

   public PlannerNodeData(int nodeId, int parentNodeId, FootstepNode node, RigidBodyTransform pose,
                          BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      this(nodeId, parentNodeId, node.getXIndex(), node.getYIndex(), node.getYawIndex(), node.getRobotSide(), pose, rejectionReason);
   }

   public PlannerNodeData(int nodeId, int parentNodeId, int xIndex, int yIndex, int yawIndex, RobotSide robotSide, RigidBodyTransform pose,
                          BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      this.footstepNode = new FootstepNode(xIndex, yIndex, yawIndex, robotSide);
      this.nodeId = nodeId;
      this.parentNodeId = parentNodeId;
      this.pose.set(pose);
      this.rejectionReason = rejectionReason;

      hashCode = footstepNode.hashCode();
   }

   public PlannerNodeData(FootstepNodeDataMessage message)
   {
      this(message.getNodeId(), message.getParentNodeId(), message.getFootstepNode().getXIndex(), message.getFootstepNode().getYIndex(),
           message.getFootstepNode().getYawIndex(), RobotSide.fromByte(message.getFootstepNode().getRobotSide()),
           new RigidBodyTransform(message.getOrientation(), message.getPosition()),
           BipedalFootstepPlannerNodeRejectionReason.fromByte(message.getBipedalFootstepPlannerNodeRejectionReason()));
   }

   public FootstepNode getFootstepNode()
   {
      return footstepNode;
   }

   public int getNodeId()
   {
      return nodeId;
   }

   public int getParentNodeId()
   {
      return parentNodeId;
   }

   public Pose3DReadOnly getNodePose()
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

      message.setNodeId(getNodeId());
      message.setParentNodeId(getParentNodeId());
      message.getFootstepNode().setXIndex(getFootstepNode().getXIndex());
      message.getFootstepNode().setYIndex(getFootstepNode().getYIndex());
      message.getFootstepNode().setYawIndex(getFootstepNode().getYawIndex());
      message.getFootstepNode().setRobotSide(getFootstepNode().getRobotSide().toByte());
      message.getPosition().set(getNodePose().getPosition());
      message.getOrientation().set(getNodePose().getOrientation());
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
