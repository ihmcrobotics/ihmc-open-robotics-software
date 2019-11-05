package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage;
import controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class PlannerLatticeMap
{
   private final Set<LatticeNode> occupiedCells = new HashSet<>();

   public PlannerLatticeMap()
   {
   }

   public PlannerLatticeMap(FootstepPlannerLatticeMapMessage message)
   {
      this();
      set(message);
   }

   public void addLatticeNode(LatticeNode occupiedCell)
   {
      occupiedCells.add(occupiedCell);
   }

   public Collection<LatticeNode> getLatticeNodes()
   {
      return occupiedCells;
   }

   public void clear()
   {
      occupiedCells.clear();
   }

   public void set(PlannerLatticeMap occupancyMap)
   {
      occupiedCells.clear();
      append(occupancyMap);
   }

   public void append(PlannerLatticeMap latticeMap)
   {
      occupiedCells.addAll(latticeMap.getLatticeNodes());
   }

   public void set(FootstepPlannerLatticeMapMessage message)
   {
      occupiedCells.clear();
      for (FootstepPlannerLatticeNodeMessage nodeMessage : message.getLatticeNodes())
         occupiedCells.add(new LatticeNode(nodeMessage.getXIndex(), nodeMessage.getYIndex(), nodeMessage.getYawIndex()));
   }

   public FootstepPlannerLatticeMapMessage getAsMessage()
   {
      FootstepPlannerLatticeMapMessage message = new FootstepPlannerLatticeMapMessage();
      getAsMessage(message);
      return message;
   }

   public void getAsMessage(FootstepPlannerLatticeMapMessage messageToPack)
   {
      for (LatticeNode occupiedCell : occupiedCells)
      {
         FootstepPlannerLatticeNodeMessage nodeMessage = messageToPack.getLatticeNodes().add();
         nodeMessage.setXIndex(occupiedCell.getXIndex());
         nodeMessage.setYIndex(occupiedCell.getYIndex());
         nodeMessage.setYawIndex(occupiedCell.getYawIndex());
      }
   }
}
