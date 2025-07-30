package us.ihmc.perception.gpuHeightMap.worldModel;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.nio.ShortBuffer;

public class Chunk
{
   public static final double LATTICE_WIDTH = 2.0;

   private double centerX;
   private double centerY;
   private float heightMapOffset;
   private double cellSize;
   private float scalingFactor;
   private Mat chunk;
   private int cellsPerAxis;

   public Chunk(double cellSize, double centerX, double centerY, float heightMapOffset, float scalingFactor)
   {
      this.cellSize = cellSize;
      this.centerX = centerX;
      this.centerY = centerY;
      this.heightMapOffset = heightMapOffset;
      this.scalingFactor = scalingFactor;

      int centerIndex = HeightMapTools.computeCenterIndex(Chunk.LATTICE_WIDTH, cellSize);
      cellsPerAxis = 2 * centerIndex + 1;

      chunk = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);

      setDefaultHeight(cellsPerAxis);
   }

   public Chunk(HeightMapMessage heightMapMessage)
   {
      this.centerX = heightMapMessage.getGridCenterX();
      this.centerY = heightMapMessage.getGridCenterY();
      this.heightMapOffset = (float) heightMapMessage.getHeightOffset();
      this.cellSize = (float) heightMapMessage.getCellSizeInMeters();
      this.scalingFactor = (float) heightMapMessage.getHeightScaleFactor();

      this.chunk = HeightMapMessageTools.unpackMessageToMat(heightMapMessage);

      int centerIndex = HeightMapTools.computeCenterIndex(Chunk.LATTICE_WIDTH, heightMapMessage.getXyResolution());
      cellsPerAxis = 2 * centerIndex + 1;

      setDefaultHeight(cellsPerAxis);
   }

   private void setDefaultHeight(int cellsPerAxis)
   {
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

   public float getHeightMapOffset()
   {
      return heightMapOffset;
   }

   public double getCellSize()
   {
      return cellSize;
   }

   public float getScalingFactor()
   {
      return scalingFactor;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public void setCenterX(double centerX)
   {
      this.centerX = centerX;
   }

   public void setCenterY(double centerY)
   {
      this.centerY = centerY;
   }

   public void setHeightMapOffset(float heightMapOffset)
   {
      this.heightMapOffset = heightMapOffset;
   }

   public void setCellSize(double cellSize)
   {
      this.cellSize = cellSize;
   }

   public void setScalingFactor(float scalingFactor)
   {
      this.scalingFactor = scalingFactor;
   }

   public void setChunk(Mat chunk)
   {
      this.chunk = chunk;
   }

   public void setCellsPerAxis(int cellsPerAxis)
   {
      this.cellsPerAxis = cellsPerAxis;
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
