package us.ihmc.perception.gpuHeightMap.worldModel;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.nio.ShortBuffer;

/**
 * This class represents a spot in the world that we are modeling.
 * A "chunk" is represented in a similar fashion to how exploring a minecraft world works
 * As parts of the map come into the robots field of view, we create that chunk and send it to be visualized
 * The chunk is represented on a world grid.
 * <p> Some more notes about a chunk: </p>
 * <ui>
 * <li> The origin of the chunk represents one corner of the chunk.</li>
 * <li> The chunks don't overlap with each other. The edges meet nicely for each chunk</li>
 * <li> A chunk is very similar to the height map, but we don't rely on in the center the same way, we use the origin as the corner</li>
 * </ui>
 */
public class Chunk
{
   /**
    * Width of a chunk in meters
    */
   public static final double CHUNK_WIDTH = 1.0;

   private double originX;
   private double originY;
   private double heightMapOffset;
   private double scalingFactor;
   private double cellSize;

   private int cellsPerAxis;

   private Mat chunk;
   private short[] chunkHeights;

   public Chunk(double originX, double originY, double cellSize, int cellsPerAxis, double heightMapOffset, double scalingFactor)
   {
      this.originX = originX;
      this.originY = originY;
      this.cellSize = cellSize;
      this.cellsPerAxis = cellsPerAxis;
      this.heightMapOffset = heightMapOffset;
      this.scalingFactor = scalingFactor;

      int totalCells = this.cellsPerAxis * this.cellsPerAxis;
      chunk = new Mat(this.cellsPerAxis, this.cellsPerAxis, opencv_core.CV_16UC1);
      chunkHeights = new short[totalCells];

      setDefaultHeight(this.cellsPerAxis);
   }

   /**
    * This is meant to be used on the receiving side when you've received a {@link ChunkMessage}, then we "download" all that data to use later.
    */
   public Chunk(ChunkMessage chunkMessage)
   {
      this.originX = chunkMessage.getOriginX();
      this.originY = chunkMessage.getOriginY();
      this.cellSize = chunkMessage.getCellSizeInMeters();
      this.cellsPerAxis = chunkMessage.getCellsPerAxis();
      this.heightMapOffset = (float) chunkMessage.getHeightOffset();
      this.scalingFactor = chunkMessage.getHeightScaleFactor();

      this.chunk = HeightMapMessageTools.unpackMessageToMat(chunkMessage);

      setDefaultHeight(cellsPerAxis);
   }

   /**
    * We shouldn't have t odo this, but there is a memory problem where the new Mats are pointing to an old address or something
    * and that is causing the new data to be filled with old data. Even though its a new object.
    * So... TODO remove this hehe
    */
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

      if (i >= 0 && i < cellsPerAxis && j >= 0 && j < cellsPerAxis)
      {
         chunkHeights[i * cellsPerAxis + j] = height;
      }
   }

   public void commitHeightsToMat()
   {
      ShortBuffer buffer = chunk.createBuffer();
      buffer.put(chunkHeights);
   }

   public static int generateHashForChunk(double xIndex, double yIndex)
   {
      int ix = (int) Math.floor(xIndex);
      int iy = (int) Math.floor(yIndex);

      return 13 * ix + 17 * iy;
   }
}
