package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class PlannerNodeDataList
{
   private final List<PlannerNodeData> nodeDataList = new ArrayList<>();

   public void addNode(PlannerNodeData parentNode, RobotSide newNodeSide, Pose3DReadOnly newNodePose, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      int parentIndex = nodeDataList.indexOf(parentNode);
      PlannerNodeData newNode = new PlannerNodeData(parentIndex, newNodeSide, newNodePose, rejectionReason);
      nodeDataList.add(newNode);
   }
}
