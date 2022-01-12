package us.ihmc.footstepPlanning.bodyPath;

public class BodyPathLatticePoint
{
   public static final double gridSizeXY = 0.15;

   private final int xIndex;
   private final int yIndex;
   private final int hashCode;

   public BodyPathLatticePoint(double x, double y)
   {
      this((int) Math.round(x / gridSizeXY), (int) Math.round(y / gridSizeXY));
   }

   public BodyPathLatticePoint(int xIndex, int yIndex)
   {
      this.xIndex = xIndex;
      this.yIndex = yIndex;
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

   public double getX()
   {
      return BodyPathLatticePoint.gridSizeXY * getXIndex();
   }

   public double getY()
   {
      return BodyPathLatticePoint.gridSizeXY * getYIndex();
   }

   private static int computeHashCode(BodyPathLatticePoint cell)
   {
      int result = 1;
      int primeX = 13;
      int primeY = 31;
      result += primeX * cell.xIndex;
      result += primeY * cell.yIndex;

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
      BodyPathLatticePoint other = (BodyPathLatticePoint) obj;

      return (xIndex == other.xIndex) && (yIndex == other.yIndex);
   }

   @Override
   public String toString()
   {
      return "(" + xIndex + ", " + yIndex + ")";
   }
}
