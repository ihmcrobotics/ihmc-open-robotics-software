package us.ihmc.perception.gpuHeightMap.worldModel;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.nio.ShortBuffer;

public class Chunk
{
   public static final double LATTICE_WIDTH = 2.0;
   private final double centerX;
   private final double centerY;
   private final Mat chunk;

   public Chunk(double resolution, double centerX, double centerY)
   {
      this.centerX = centerX;
      this.centerY = centerY;

      int centerIndex = HeightMapTools.computeCenterIndex(Chunk.LATTICE_WIDTH, resolution);
      int cellsPerAxis = 2 * centerIndex + 1;

      chunk = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);

      // Set default height of the Mat
      int totalCells = cellsPerAxis * cellsPerAxis;
      short[] heights = new short[totalCells];
      ShortBuffer buffer = chunk.createBuffer();
      buffer.put(heights);
   }

   public Mat getChunk()
   {
      return chunk;
   }

   public double getCenterX()
   {
      return centerX;
   }

   public double getCenterY()
   {
      return centerY;
   }

   public void setHeightAt(double xCord, double yCord, short height, double resolution, int centerIndex)
   {
      int i = HeightMapTools.coordinateToIndex(xCord, centerX, resolution, centerIndex);
      int j = HeightMapTools.coordinateToIndex(yCord, centerY, resolution, centerIndex);

      if (i >= 0 && i < chunk.rows() && j >= 0 && j < chunk.cols())
      {
         chunk.ptr(i, j).putShort(height);
      }
   }

   public static int generateHashForChunk(double xIndex, double yIndex)
   {
      int ix = (int) Math.floor(xIndex);
      int iy = (int) Math.floor(yIndex);

      return 13 * ix + 17 * iy;
   }
}
