package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class PlannerNodeDataList
{
   private boolean isFootstepGraph = false;
   private final List<PlannerNodeData> nodeDataList = new ArrayList<>();

   public PlannerNodeDataList()
   {}

   public PlannerNodeDataList(FootstepNodeDataListMessage message)
   {
      this();
      set(message);
   }

   public void setIsFootstepGraph(boolean isFootstepGraph)
   {
      this.isFootstepGraph = isFootstepGraph;
   }

   public boolean isFootstepGraph()
   {
      return isFootstepGraph;
   }

   public void clear()
   {
      nodeDataList.clear();
   }

   public int size()
   {
      return nodeDataList.size();
   }

   public List<PlannerNodeData> getNodeData()
   {
      return nodeDataList;
   }

   public PlannerNodeData getDataForNode(FootstepNode node)
   {
      if (node == null)
         return null;

      return nodeDataList.get(nodeDataList.indexOf(node.getLatticeNode()));
   }


   public PlannerNodeData addNode(int xIndex, int yIndex, int yawIndex, RobotSide newNodeSide, Pose3DReadOnly newNodePose)
   {
      return addNode(-1, nodeDataList.size(), xIndex, yIndex, yawIndex, newNodeSide, newNodePose, null);
   }

   public PlannerNodeData addNode(PlannerNodeData parentNode, int nodeIndex, LatticeNode latticeNode, RobotSide newNodeSide,
                                  Pose3DReadOnly newNodePose, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      return addNode(parentNode, nodeIndex, latticeNode.getXIndex(), latticeNode.getYIndex(), latticeNode.getYawIndex(), newNodeSide, newNodePose,
                     rejectionReason);
   }

   public PlannerNodeData addNode(PlannerNodeData parentNode, int nodeIndex, int xIndex, int yIndex, int yawIndex, RobotSide newNodeSide,
                                  Pose3DReadOnly newNodePose, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      return addNode(nodeDataList.indexOf(parentNode), nodeIndex, xIndex, yIndex, yawIndex, newNodeSide, newNodePose, rejectionReason);
   }

   public PlannerNodeData addNode(int parentNodeIndex, int nodeIndex, int xIndex, int yIndex, int yawIndex, RobotSide newNodeSide, Pose3DReadOnly newNodePose,
                                  BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      return addNode(new PlannerNodeData(parentNodeIndex, nodeIndex, xIndex, yIndex, yawIndex, newNodeSide, newNodePose, rejectionReason));
   }

   public PlannerNodeData addNode(int parentNodeIndex, int nodeIndex, LatticeNode latticeNode, RobotSide newNodeSide, Pose3DReadOnly newNodePose,
                                  BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      return addNode(parentNodeIndex, nodeIndex, latticeNode.getXIndex(), latticeNode.getYIndex(), latticeNode.getYawIndex(), newNodeSide, newNodePose,
                     rejectionReason);
   }

   public PlannerNodeData addNode(PlannerNodeData nodeData)
   {
      nodeDataList.add(nodeData);
      return nodeData;
   }

   public FootstepNodeDataListMessage getAsMessage()
   {
      FootstepNodeDataListMessage message = new FootstepNodeDataListMessage();
      getAsMessage(message);
      return message;
   }

   public void getAsMessage(FootstepNodeDataListMessage message)
   {
      message.setIsFootstepGraph(isFootstepGraph());
      for (PlannerNodeData nodeData : nodeDataList)
         nodeData.getAsMessage(message.getNodeData().add());
   }

   public void set(FootstepNodeDataListMessage message)
   {
      clear();
      setIsFootstepGraph(message.getIsFootstepGraph());
      for (FootstepNodeDataMessage nodeDataMessage : message.getNodeData())
      {
         addNode(new PlannerNodeData(nodeDataMessage));
      }

      getNodeData().sort(Comparator.comparingInt(PlannerNodeData::getNodeId));
   }

}
