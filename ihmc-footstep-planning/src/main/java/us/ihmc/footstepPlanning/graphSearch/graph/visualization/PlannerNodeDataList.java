package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
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

   public List<PlannerNodeData> getNodeData()
   {
      return nodeDataList;
   }

   public void addNode(RobotSide newNodeSide, Pose3DReadOnly newNodePose)
   {
      addNode(-1, newNodeSide, newNodePose, null);
   }

   public void addNode(PlannerNodeData parentNode, RobotSide newNodeSide, Pose3DReadOnly newNodePose, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      addNode(nodeDataList.indexOf(parentNode), newNodeSide, newNodePose, rejectionReason);
   }

   public void addNode(int parentNodeIndex, RobotSide newNodeSide, Pose3DReadOnly newNodePose, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      addNode(new PlannerNodeData(parentNodeIndex, newNodeSide, newNodePose, rejectionReason));
   }

   public void addNode(PlannerNodeData nodeData)
   {
      nodeDataList.add(nodeData);
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

      getNodeData().sort(Comparator.comparingInt(PlannerNodeData::getParentNodeId));
   }

}
