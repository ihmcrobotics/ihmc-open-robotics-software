package us.ihmc.sensorProcessing.globalHeightMap;

import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class GlobalMapCell extends HeightMapData
{
   private final int centerX, centerY;
   private int hashCode;

   public GlobalMapCell(double resolution, double centerX, double centerY)
   {
      super(resolution, GlobalLattice.latticeWidth, centerX, centerY);

      this.centerX = GlobalLattice.toIndex(centerX);
      this.centerY = GlobalLattice.toIndex(centerY);
      hashCode = GlobalLattice.hashCodeOfCell(centerX, centerY);
   }


   @Override
   public int hashCode()
   {
      return hashCode;
   }

   public int getCenterX()
   {
      return centerX;
   }

   public int getCenterY()
   {
      return centerY;
   }
}
