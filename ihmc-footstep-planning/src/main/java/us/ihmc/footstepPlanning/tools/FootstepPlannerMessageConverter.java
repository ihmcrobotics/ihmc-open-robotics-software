package us.ihmc.footstepPlanning.tools;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage;
import controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerLatticeMap;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeData;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeDataList;

public class FootstepPlannerMessageConverter
{
   public static FootstepPlannerLatticeMapMessage convertExpandedNodesToMessage(int planId, PlannerLatticeMap expandedNodes)
   {
      FootstepPlannerLatticeMapMessage latticeMapMessage = new FootstepPlannerLatticeMapMessage();

      latticeMapMessage.setPlanId(planId);

      for (FootstepNode stageCell : expandedNodes.getLatticeNodes())
      {
         if (latticeMapMessage.getLatticeNodes().size() == 10000)
            break;

         FootstepPlannerLatticeNodeMessage nodeMessage = latticeMapMessage.getLatticeNodes().add();
         nodeMessage.setXIndex(stageCell.getXIndex());
         nodeMessage.setYIndex(stageCell.getYIndex());
         nodeMessage.setYawIndex(stageCell.getYawIndex());
         nodeMessage.setRobotSide(stageCell.getRobotSide().toByte());
         nodeMessage.setNodeIndex(stageCell.getNodeIndex());
      }

      return latticeMapMessage;
   }

   public static FootstepNodeDataListMessage convertFullGraphToMessage(int planId, PlannerNodeDataList fullGraph)
   {
      FootstepNodeDataListMessage message = new FootstepNodeDataListMessage();
      message.setPlanId(planId);
      message.setIsFootstepGraph(true);

      for (PlannerNodeData nodeData : fullGraph.getNodeData())
      {
         if (message.getNodeData().size() == 200000)
            break;

         FootstepNodeDataMessage nodeDataMessage = message.getNodeData().add();
         nodeData.getAsMessage(nodeDataMessage);
         nodeDataMessage.getFootstepNode().setNodeIndex(nodeData.getNodeId());
      }

      return message;
   }
}
