package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage;
import controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class PlannerLatticeMap
{
   private final Set<FootstepNode> nodes = new HashSet<>();

   public PlannerLatticeMap()
   {
   }

   public PlannerLatticeMap(FootstepPlannerLatticeMapMessage message)
   {
      this();
      set(message);
   }

   public void addFootstepNode(FootstepNode footstepNode)
   {
      nodes.add(footstepNode);
   }

   public Collection<FootstepNode> getLatticeNodes()
   {
      return nodes;
   }

   public void clear()
   {
      nodes.clear();
   }

   public void set(PlannerLatticeMap occupancyMap)
   {
      nodes.clear();
      append(occupancyMap);
   }

   public void append(PlannerLatticeMap latticeMap)
   {
      nodes.addAll(latticeMap.getLatticeNodes());
   }

   public void set(FootstepPlannerLatticeMapMessage message)
   {
      nodes.clear();
      for (FootstepPlannerLatticeNodeMessage nodeMessage : message.getLatticeNodes())
      {
         FootstepNode node = new FootstepNode(nodeMessage.getXIndex(), nodeMessage.getYIndex(), nodeMessage.getYawIndex(), RobotSide.fromByte(nodeMessage.getRobotSide()));
         node.setNodeIndex(nodeMessage.getNodeIndex());
         nodes.add(node);
      }
   }

   public FootstepPlannerLatticeMapMessage getAsMessage()
   {
      FootstepPlannerLatticeMapMessage message = new FootstepPlannerLatticeMapMessage();
      getAsMessage(message);
      return message;
   }

   public void getAsMessage(FootstepPlannerLatticeMapMessage messageToPack)
   {
      for (FootstepNode occupiedCell : nodes)
      {
         FootstepPlannerLatticeNodeMessage nodeMessage = messageToPack.getLatticeNodes().add();
         nodeMessage.setXIndex(occupiedCell.getXIndex());
         nodeMessage.setYIndex(occupiedCell.getYIndex());
         nodeMessage.setYawIndex(occupiedCell.getYawIndex());
         nodeMessage.setRobotSide(occupiedCell.getRobotSide().toByte());
         nodeMessage.setNodeIndex(occupiedCell.getNodeIndex());
      }
   }
}
