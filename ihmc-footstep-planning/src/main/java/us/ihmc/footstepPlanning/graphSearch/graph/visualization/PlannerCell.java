package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import toolbox_msgs.msg.dds.FootstepPlannerCellMessage;

public class PlannerCell
{
   private final int xIndex;
   private final int yIndex;

   private final int hashCode;

   public PlannerCell(int xIndex, int yIndex)
   {
      this.xIndex = xIndex;
      this.yIndex = yIndex;
      hashCode = computeHashCode(this);
   }

   public PlannerCell(FootstepPlannerCellMessage message)
   {
      this(message.getXIndex(), message.getYIndex());
   }

   public int getXIndex()
   {
      return xIndex;
   }

   public int getYIndex()
   {
      return yIndex;
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   private static int computeHashCode(PlannerCell cell)
   {
      int result = 1;
      int prime = 31;
      result += prime * cell.xIndex;
      result += prime * cell.yIndex;
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == null)
         return false;
      else if (obj == this)
         return true;

      if (getClass() != obj.getClass())
         return false;
      PlannerCell other = (PlannerCell) obj;
      return xIndex == other.xIndex && yIndex == other.yIndex;
   }

   public void getAsMessage(FootstepPlannerCellMessage messageToPack)
   {
      messageToPack.setXIndex(xIndex);
      messageToPack.setYIndex(yIndex);
   }
}
