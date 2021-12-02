package us.ihmc.footstepPlanning.bodyPath;

public class BodyPathLatticePoint
{
   public static final double gridSizeXY = 0.15;
   public static final int yawDivisions = 8; // 16;
   public static final double gridSizeYaw = 2.0 * Math.PI / yawDivisions;

   private final int xIndex;
   private final int yIndex;
   private final int yawIndex;
   private final int hashCode;

   public BodyPathLatticePoint(double x, double y, double yaw)
   {
      this((int) Math.round(x / gridSizeXY), (int) Math.round(y / gridSizeXY), Math.floorMod((int) (Math.round((yaw) / gridSizeYaw)), yawDivisions));
   }

   public BodyPathLatticePoint(int xIndex, int yIndex, int yawIndex)
   {
      this.xIndex = xIndex;
      this.yIndex = yIndex;

      if (yawIndex < 0)
         this.yawIndex = yawIndex + yawDivisions;
      else if (yawIndex >= yawDivisions)
         this.yawIndex = yawIndex - yawDivisions;
      else
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
      return BodyPathLatticePoint.gridSizeXY * getXIndex();
   }

   public double getY()
   {
      return BodyPathLatticePoint.gridSizeXY * getYIndex();
   }

   public double getYaw()
   {
      return gridSizeYaw * getYawIndex();
   }

   private static int computeHashCode(BodyPathLatticePoint cell)
   {
      int result = 1;
      int primeX = 13;
      int primeY = 31;
      int primeYaw = 17;
      result += primeX * cell.xIndex;
      result += primeY * cell.yIndex;
      result += primeYaw * cell.yawIndex;

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

      return (xIndex == other.xIndex) && (yIndex == other.yIndex) && (yawIndex == other.yawIndex);
   }

   @Override
   public String toString()
   {
      return "(" + xIndex + ", " + yIndex + ", "+ yawIndex + ")";
   }
}
