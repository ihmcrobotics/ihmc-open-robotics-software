package us.ihmc.robotics.occupancyGrid;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;

public class OccupancyGridCell
{
   private DoubleProvider occupancyThreshold = () -> 1;
   private DoubleProvider occupancyDecayRate = () -> 1.0;
   private boolean isOccupied = false;
   private double numberOfHits = 0.0;

   private final int xIndex;
   private final int yIndex;

   private final int hashCode;

   public OccupancyGridCell(int xIndex, int yIndex)
   {
      this.xIndex = xIndex;
      this.yIndex = yIndex;

      hashCode = computeHashCode(this);
   }

   public OccupancyGridCell(int xIndex, int yIndex, DoubleProvider occupancyThreshold, DoubleProvider occupancyDecayRate)
   {
      this(xIndex, yIndex);
      setOccupancyThreshold(occupancyThreshold);
      setOccupancyDecayRate(occupancyDecayRate);
   }

   public boolean getIsOccupied()
   {
      return isOccupied;
   }

   public int getXIndex()
   {
      return xIndex;
   }

   public int getYIndex()
   {
      return yIndex;
   }

   public void reset()
   {
      isOccupied = false;
      numberOfHits = 0.0;
   }

   public void setOccupancyThreshold(DoubleProvider occupancyThreshold)
   {
      this.occupancyThreshold = occupancyThreshold;
   }

   public void setOccupancyDecayRate(DoubleProvider occupancyDecayRate)
   {
      this.occupancyDecayRate = occupancyDecayRate;
   }

   public void update()
   {
      if (occupancyDecayRate.getValue() >= 1.0)
         return;

      numberOfHits *= occupancyDecayRate.getValue();

      updateOccupancyEstimate();
   }

   public void registerHit()
   {
      numberOfHits += 1.0;
      updateOccupancyEstimate();
   }

   private void updateOccupancyEstimate()
   {
      isOccupied = numberOfHits >= occupancyThreshold.getValue();
   }

   public static int computeHashCode(OccupancyGridCell cell)
   {
      return computeHashCode(cell.xIndex, cell.yIndex);
   }

   public static int computeHashCode(int xIndex, int yIndex)
   {
      int result = 1;
      int prime = 31;
      result += prime * xIndex;
      result += prime * yIndex;

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
      OccupancyGridCell other = (OccupancyGridCell) obj;
      if (xIndex != other.xIndex)
         return false;
      if (yIndex != other.yIndex)
         return false;
      return true;
   }
}
