package us.ihmc.sensorProcessing.globalHeightMap;

import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class GlobalMapTile extends HeightMapData
{
   private final int centerX, centerY;
   private int hashCode;

   public GlobalMapTile(double resolution, double centerX, double centerY)
   {
      super(resolution, GlobalLattice.latticeWidth, centerX, centerY);

      this.centerX = GlobalLattice.toIndex(centerX);
      this.centerY = GlobalLattice.toIndex(centerY);
      hashCode = GlobalLattice.hashCodeOfTilePositions(centerX, centerY);
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

   public HeightMapData getHeightMapDataForPublishing()
   {
//      HeightMapData heightMapDataForPublishing = new HeightMapData(getGridResolutionXY(),
//                                                                   getGridSizeXY(), getGridCenter().getX(), getGridCenter().getY());
//
//      return heightMapDataForPublishing;
      return this;
   }
}
