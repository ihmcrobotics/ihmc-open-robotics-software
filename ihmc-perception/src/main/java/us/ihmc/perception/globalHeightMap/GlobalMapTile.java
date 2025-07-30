package us.ihmc.perception.globalHeightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.nio.ShortBuffer;

public class GlobalMapTile
{
   private final double tileCenterX;
   private final double tileCenterY;
   private final int hashCode;
   private final Mat tile;

   public GlobalMapTile(double resolution, double centerX, double centerY)
   {
      tileCenterX = centerX;
      tileCenterY = centerY;
      hashCode = GlobalLattice.hashCodeOfTilePositions(centerX, centerY);

      int centerIndexLocal = HeightMapTools.computeCenterIndex(GlobalLattice.latticeWidth, resolution);
      int cellsPerAxisLocal = 2 * centerIndexLocal + 1;

      tile = new Mat(cellsPerAxisLocal, cellsPerAxisLocal, opencv_core.CV_16UC1);

      int totalCells = cellsPerAxisLocal * cellsPerAxisLocal;
      short[] heightsAsFloats = new short[totalCells];
      ShortBuffer buffer = tile.createBuffer();
      buffer.put(heightsAsFloats);

   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   public double getCenterX()
   {
      return tileCenterX;
   }

   public double getCenterY()
   {
      return tileCenterY;
   }

   public void setHeightAt(double xCord, double yCord, short height, double resolution, int centerIndex)
   {
      int i = HeightMapTools.coordinateToIndex(xCord, tileCenterX, resolution, centerIndex);
      int j = HeightMapTools.coordinateToIndex(yCord, tileCenterY, resolution, centerIndex);

      if (i >= 0 && i < tile.rows() && j >= 0 && j < tile.cols())
      {
         tile.ptr(i, j).putShort(height); // Store value into tile
      }
   }

   public Mat getTile()
   {
      return tile;
   }
}
