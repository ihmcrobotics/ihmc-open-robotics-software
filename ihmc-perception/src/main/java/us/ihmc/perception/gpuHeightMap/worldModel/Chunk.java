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

   private double originX;
   private double originY;
   private double heightMapOffset;
   private double cellSize;
   private double scalingFactor;
   private Mat chunk;
   private int cellsPerAxis;

   public Chunk(double cellSize, double originX, double originY, double heightMapOffset, double scalingFactor)
   {
      this.cellSize = cellSize;
      this.originX = originX;
      this.originY = originY;
      this.heightMapOffset = heightMapOffset;
      this.scalingFactor = scalingFactor;

      int centerIndex = HeightMapTools.computeCenterIndex(Chunk.LATTICE_WIDTH, cellSize);
      cellsPerAxis = 2 * centerIndex;

      chunk = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);

      setDefaultHeight(cellsPerAxis);
   }

   public Chunk(HeightMapMessage heightMapMessage)
   {
      this.originX = heightMapMessage.getGridCenterX();
      this.originY = heightMapMessage.getGridCenterY();
      this.heightMapOffset = (float) heightMapMessage.getHeightOffset();
      this.cellSize = heightMapMessage.getCellSizeInMeters();
      this.scalingFactor = heightMapMessage.getHeightScaleFactor();

      this.chunk = HeightMapMessageTools.unpackMessageToMat(heightMapMessage);

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapMessage.getGridSizeXy(), heightMapMessage.getXyResolution());
      cellsPerAxis = 2 * centerIndex;

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

   public double getOriginX()
   {
      return originX;
   }

   public double getOriginY()
   {
      return originY;
   }

   public double getHeightMapOffset()
   {
      return heightMapOffset;
   }

   public double getCellSize()
   {
      return cellSize;
   }

   public double getScalingFactor()
   {
      return scalingFactor;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public void setOriginX(double originX)
   {
      this.originX = originX;
   }

   public void setOriginY(double originY)
   {
      this.originY = originY;
   }

   public void setHeightMapOffset(float heightMapOffset)
   {
      this.heightMapOffset = heightMapOffset;
   }

   public void setCellSize(double cellSize)
   {
      this.cellSize = cellSize;
   }

   public void setScalingFactor(double scalingFactor)
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

   public void setHeightAt(double xCord, double yCord, short height, double resolution)
   {
      int i = HeightMapTools.coordinateToIndex(xCord, this.originX, resolution);
      int j = HeightMapTools.coordinateToIndex(yCord, this.originY, resolution);

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
