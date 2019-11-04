package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class PlannerOccupancyMap
{
   private final Set<PlannerCell> occupiedCells = new HashSet<>();

   public PlannerOccupancyMap()
   {
   }

   public PlannerOccupancyMap(FootstepPlannerOccupancyMapMessage message)
   {
      this();
      set(message);
   }

   public void addOccupiedCell(PlannerCell occupiedCell)
   {
      occupiedCells.add(occupiedCell);
   }

   public Collection<PlannerCell> getOccupiedCells()
   {
      return occupiedCells;
   }

   public void clear()
   {
      occupiedCells.clear();
   }

   public void set(FootstepPlannerOccupancyMapMessage message)
   {
      occupiedCells.clear();
      for (FootstepPlannerCellMessage cellMessage : message.getOccupiedCells())
         occupiedCells.add(new PlannerCell(cellMessage));
   }

   public void getAsMessage(FootstepPlannerOccupancyMapMessage messageToPack)
   {
      for (PlannerCell occupiedCell : occupiedCells)
         occupiedCell.getAsMessage(messageToPack.getOccupiedCells().add());
   }
}
