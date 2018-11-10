package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

class PlannerCell
{
   int xIndex;
   int yIndex;

   PlannerCell(int xIndex, int yIndex)
   {
      this.xIndex = xIndex;
      this.yIndex = yIndex;
   }

   @Override
   public int hashCode()
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
}
