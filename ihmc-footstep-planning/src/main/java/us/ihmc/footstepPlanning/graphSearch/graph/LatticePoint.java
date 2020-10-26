package us.ihmc.footstepPlanning.graphSearch.graph;

public class LatticePoint
{
   public static final double gridSizeXY = 0.05;

   public static final int yawDivisions = 36;
   public static final double gridSizeYaw = 2.0 * Math.PI / yawDivisions;

   private final int xIndex;
   private final int yIndex;
   private final int yawIndex;

   private final int hashCode;

   public LatticePoint(double x, double y, double yaw)
   {
      this((int) Math.round(x / gridSizeXY), (int) Math.round(y / gridSizeXY), Math.floorMod((int) (Math.round((yaw) / gridSizeYaw)), yawDivisions));
   }

   public LatticePoint(int xIndex, int yIndex, int yawIndex)
   {
      this.xIndex = xIndex;
      this.yIndex = yIndex;
      this.yawIndex = yawIndex;
      hashCode = computeHashCode(this);
   }

   public int getXIndex()
   {
      return xIndex;
   }

   public int getYIndex()
   {
      return yIndex;
   }

   public int getYawIndex()
   {
      return yawIndex;
   }

   public double getX()
   {
      return LatticePoint.gridSizeXY * getXIndex();
   }

   public double getY()
   {
      return LatticePoint.gridSizeXY * getYIndex();
   }

   public double getYaw()
   {
      return LatticePoint.gridSizeYaw * getYawIndex();
   }

   private static int computeHashCode(LatticePoint cell)
   {
      int result = 1;
      int prime = 31;
      result += prime * cell.xIndex;
      result += prime * cell.yIndex;
      result += prime * cell.yawIndex;
      return result;
   }

   @Override
   public int hashCode()
   {
      return hashCode;
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
      LatticePoint other = (LatticePoint) obj;
      if (xIndex != other.xIndex)
         return false;
      if (yIndex != other.yIndex)
         return false;
      if (yawIndex != other.yawIndex)
         return false;
      return true;
   }

}
