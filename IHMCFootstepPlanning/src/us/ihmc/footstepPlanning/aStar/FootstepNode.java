package us.ihmc.footstepPlanning.aStar;

public class FootstepNode
{
   public static final double gridSizeX = 0.05;
   public static final double gridSizeY = 0.05;

   private final int xIndex;
   private final int yIndex;

   public FootstepNode(double x, double y)
   {
      xIndex = (int) Math.round(x / gridSizeX);
      yIndex = (int) Math.round(y / gridSizeY);
   }

   public double getX()
   {
      return gridSizeX * xIndex;
   }

   public double getY()
   {
      return gridSizeY * yIndex;
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
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      FootstepNode other = (FootstepNode) obj;
      if (xIndex != other.xIndex)
         return false;
      if (yIndex != other.yIndex)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "Node: x=" + getX() + ", y=" + getY();
   }

}
