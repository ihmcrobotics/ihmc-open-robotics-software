package us.ihmc.perception.gpuMapping;

import perception_msgs.HeightMapMessageForController;
import us.ihmc.commons.MathTools;

import java.util.zip.Deflater;

/**
 * Packs a square crop of a {@link HeightMapData} into a {@link HeightMapMessageForController}, quantizing the heights
 * to int16 and deflate-compressing them. Decoded by {@code HeightMapCommand}.
 */
public class HeightMapForControllerEncoder
{
   public static final double DEFAULT_HEIGHT_RESOLUTION = 0.002;

   private final double cropWidth;
   private final double heightResolution;
   private final Deflater deflater = new Deflater(Deflater.BEST_SPEED);
   private byte[] cells = new byte[0];
   private byte[] compressed = new byte[0];

   public HeightMapForControllerEncoder(double cropWidth)
   {
      this(cropWidth, DEFAULT_HEIGHT_RESOLUTION);
   }

   public HeightMapForControllerEncoder(double cropWidth, double heightResolution)
   {
      this.cropWidth = cropWidth;
      this.heightResolution = heightResolution;
   }

   /**
    * @param centerX      world x of the crop center, moved inward if the crop would leave the map
    * @param centerY      world y of the crop center, moved inward if the crop would leave the map
    * @param heightOffset height that quantizes to 0, keep it near the terrain so the int16 range is not exceeded
    */
   public void encode(HeightMapData heightMapData, double centerX, double centerY, double heightOffset, HeightMapMessageForController messageToPack)
   {
      double cellSize = heightMapData.getCellSize();
      int sourceCenterIndex = heightMapData.getCenterIndex();
      int sourceCellsPerAxis = heightMapData.getCellsPerAxis();
      int cropCenterIndex = Math.min(HeightMapTools.computeCenterIndex(cropWidth, cellSize), sourceCenterIndex);
      int cropCellsPerAxis = 2 * cropCenterIndex + 1;

      int xCenter = HeightMapTools.coordinateToIndex(centerX, heightMapData.getGridCenter().getX(), cellSize, sourceCenterIndex);
      int yCenter = HeightMapTools.coordinateToIndex(centerY, heightMapData.getGridCenter().getY(), cellSize, sourceCenterIndex);
      xCenter = MathTools.clamp(xCenter, cropCenterIndex, sourceCellsPerAxis - 1 - cropCenterIndex);
      yCenter = MathTools.clamp(yCenter, cropCenterIndex, sourceCellsPerAxis - 1 - cropCenterIndex);

      float offset = (float) (Math.round(heightOffset / heightResolution) * heightResolution);
      int cellBytes = 2 * cropCellsPerAxis * cropCellsPerAxis;
      if (cells.length < cellBytes)
         cells = new byte[cellBytes];

      float[] heights = heightMapData.getHeights();
      int byteIndex = 0;
      for (int xIndex = 0; xIndex < cropCellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cropCellsPerAxis; yIndex++)
         {
            int sourceKey = HeightMapTools.indicesToKey(xCenter - cropCenterIndex + xIndex, yCenter - cropCenterIndex + yIndex, sourceCenterIndex);
            short value = quantize(heights[sourceKey], offset);
            cells[byteIndex++] = (byte) value;
            cells[byteIndex++] = (byte) (value >> 8);
         }
      }

      // zlib's worst case for incompressible input, so the deflate loop always terminates
      int compressedBound = cellBytes + (cellBytes >> 12) + (cellBytes >> 14) + 64;
      if (compressed.length < compressedBound)
         compressed = new byte[compressedBound];
      deflater.reset();
      deflater.setInput(cells, 0, cellBytes);
      deflater.finish();
      int compressedSize = 0;
      while (!deflater.finished())
         compressedSize += deflater.deflate(compressed, compressedSize, compressed.length - compressedSize);

      messageToPack.setGridCenterX(HeightMapTools.indexToCoordinate(xCenter, heightMapData.getGridCenter().getX(), cellSize, sourceCenterIndex));
      messageToPack.setGridCenterY(HeightMapTools.indexToCoordinate(yCenter, heightMapData.getGridCenter().getY(), cellSize, sourceCenterIndex));
      messageToPack.setWidthInMeters(2 * cropCenterIndex * cellSize);
      messageToPack.setCellSizeInMeters(cellSize);
      messageToPack.setCellsPerAxis(cropCellsPerAxis);
      messageToPack.setHeightOffset(offset);
      messageToPack.setHeightResolution((float) heightResolution);
      messageToPack.getCompressedHeights().clear();
      for (int i = 0; i < compressedSize; i++)
         messageToPack.getCompressedHeights().add(compressed[i]);
   }

   private short quantize(float height, float offset)
   {
      if (Float.isNaN(height))
         return HeightMapMessageForController.NO_DATA_VALUE;
      long value = Math.round((height - offset) / heightResolution);
      return (short) MathTools.clamp(value, HeightMapMessageForController.NO_DATA_VALUE + 1, Short.MAX_VALUE);
   }
}
