package us.ihmc.perception.gpuMapping.worldModel;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
import us.ihmc.perception.gpuMapping.HeightMapMessageTools;
import us.ihmc.perception.gpuMapping.HeightMapTools;

import java.nio.FloatBuffer;

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
   private double cellSize;

   private int cellsPerAxis;

   private Mat chunk;
   private float[] chunkHeights;

   public Chunk(double originX, double originY, double cellSize, int cellsPerAxis)
   {
      this.originX = originX;
      this.originY = originY;
      this.cellSize = cellSize;
      this.cellsPerAxis = cellsPerAxis;

      int totalCells = this.cellsPerAxis * this.cellsPerAxis;
      chunk = new Mat(this.cellsPerAxis, this.cellsPerAxis, opencv_core.CV_32FC1);
      chunkHeights = new float[totalCells];

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
      float[] heights = new float[totalCells];
      FloatBuffer buffer = chunk.createBuffer();
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

   public double getCellSize()
   {
      return cellSize;
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

   public void setCellSize(double cellSize)
   {
      this.cellSize = cellSize;
   }

   public void setChunk(Mat chunk)
   {
      // Deep copy here cause the original gets closed
      this.chunk = chunk.clone();
   }

   public void setCellsPerAxis(int cellsPerAxis)
   {
      this.cellsPerAxis = cellsPerAxis;
   }

   public void setHeightAt(double xCord, double yCord, float height, double resolution)
   {
      int i = HeightMapTools.coordinateToChunkIndex(xCord, this.originX, resolution);
      int j = HeightMapTools.coordinateToChunkIndex(yCord, this.originY, resolution);

      if (i >= 0 && i < cellsPerAxis && j >= 0 && j < cellsPerAxis)
      {
         chunkHeights[i * cellsPerAxis + j] = height;
      }
   }

   public void commitHeightsToMat()
   {
      FloatBuffer buffer = chunk.createBuffer();
      buffer.put(chunkHeights);
   }

   public static int generateHashForChunk(double xIndex, double yIndex)
   {
      int ix = (int) Math.floor(xIndex);
      int iy = (int) Math.floor(yIndex);

      return 13 * ix + 17 * iy;
   }
}
