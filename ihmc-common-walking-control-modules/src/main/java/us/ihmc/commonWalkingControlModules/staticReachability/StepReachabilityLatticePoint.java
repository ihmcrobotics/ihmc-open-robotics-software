package us.ihmc.commonWalkingControlModules.staticReachability;

public class StepReachabilityLatticePoint
{
   private final int xIndex;
   private final int yIndex;
   private final int zIndex;
   private final int yawIndex;
   private final int hashCode;

   public StepReachabilityLatticePoint(double x, double y, double z, double yaw, double xyzSpacing, int yawDivisions, double yawSpacing)
   {
      this((int) Math.round(x / xyzSpacing), (int) Math.round(y / xyzSpacing), (int) Math.round(z / xyzSpacing), (int) ((Math.round((yaw) / yawSpacing)) % yawDivisions));
   }

   public StepReachabilityLatticePoint(int xIndex, int yIndex, int zIndex, int yawIndex)
   {
      this.xIndex = xIndex;
      this.yIndex = yIndex;
      this.zIndex = zIndex;
      this.yawIndex = yawIndex;
      this.hashCode = computeHashCode(this);
   }

   public int getXIndex()
   {
      return xIndex;
   }

   public int getYIndex()
   {
      return yIndex;
   }

   public int getZIndex()
   {
      return zIndex;
   }

   public int getYawIndex()
   {
      return yawIndex;
   }

   public double indexDistanceFrom(StepReachabilityLatticePoint step)
   {
      double sum = 0;
      sum += Math.abs(step.getXIndex() - xIndex);
      sum += Math.abs(step.getYIndex() - yIndex);
      sum += Math.abs(step.getZIndex() - zIndex);
      sum += Math.abs(step.getYawIndex() - yawIndex);
      return sum / 4;
   }

   private static int computeHashCode(StepReachabilityLatticePoint cell)
   {
      int result = 1;
      int prime = 31;
      result += prime * cell.xIndex;
      result += prime * cell.yIndex;
      result += prime * cell.zIndex;
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
      StepReachabilityLatticePoint other = (StepReachabilityLatticePoint) obj;
      if (xIndex != other.xIndex)
         return false;
      if (yIndex != other.yIndex)
         return false;
      if (zIndex != other.zIndex)
         return false;
      if (yawIndex != other.yawIndex)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "(" + xIndex + ", " + yIndex + ", " + zIndex + ", " + yawIndex + ")";
   }
}
