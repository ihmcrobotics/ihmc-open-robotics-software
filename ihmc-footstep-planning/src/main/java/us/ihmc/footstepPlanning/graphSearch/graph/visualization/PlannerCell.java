package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepPlannerCellMessage;

public class PlannerCell
{
   private int xIndex;
   private int yIndex;

   int hashCode;

   public PlannerCell(int xIndex, int yIndex)
   {
      this.xIndex = xIndex;
      this.yIndex = yIndex;
      hashCode = computeHashCode(xIndex, yIndex);
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

   public static int computeHashCode(int xIndex, int yIndex)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + xIndex;
      result = prime * result + yIndex;

      return result;
   }

   @Override
   public boolean equals(Object other)
   {
      if (!(other instanceof PlannerCell))
         return false;

      PlannerCell otherCell = (PlannerCell) other;
      return otherCell.xIndex == xIndex && otherCell.yIndex == yIndex;
   }

   public void set(FootstepPlannerCellMessage message)
   {
      this.xIndex = message.getXIndex();
      this.yIndex = message.getYIndex();
   }

   public void getAsMessage(FootstepPlannerCellMessage messageToPack)
   {
      messageToPack.setXIndex(xIndex);
      messageToPack.setYIndex(yIndex);
   }
}
