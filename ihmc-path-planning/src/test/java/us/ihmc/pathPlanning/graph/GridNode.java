package us.ihmc.pathPlanning.graph;

/**
 * Helper class for testing search tools
 */
public class GridNode
{
   private final int x, y;

   public GridNode(int x, int y)
   {
      this.x = x;
      this.y = y;
   }

   public int getX()
   {
      return x;
   }

   public int getY()
   {
      return y;
   }

   @Override
   public int hashCode()
   {
      return 5 * x + 13 * y;
   }

   @Override
   public boolean equals(Object o)
   {
      GridNode gridNode = (GridNode) o;
      return (gridNode.x == x) && (gridNode.y == y);
   }
}
