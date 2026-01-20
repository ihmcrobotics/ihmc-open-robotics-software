package us.ihmc.perception.gpuMapping.worldModel;

import us.ihmc.perception.gpuMapping.HeightMapTools;

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
   public static final float CHUNK_WIDTH = 1.0f;

   private float originX;
   private float originY;
   private float cellSize;
   private int cellsPerAxis;
   private final float widthInMeters;

   private float[] chunkHeights;

   /**
    * The idea of the dirty field is to note if the chunk has been updated or not.
    * We say a chunk is dirty if one of the updated height values, doesn't equal the previous height value for the given cell
    */
   private boolean dirty = false;

   public Chunk(float originX, float originY, float cellSize, int cellsPerAxis)
   {
      this.originX = originX;
      this.originY = originY;
      this.cellSize = cellSize;
      this.cellsPerAxis = cellsPerAxis;
      this.widthInMeters = cellSize * cellsPerAxis;

      int totalCells = this.cellsPerAxis * this.cellsPerAxis;
      chunkHeights = new float[totalCells];
   }

   public float getOriginX()
   {
      return originX;
   }

   public float getOriginY()
   {
      return originY;
   }

   public float getCellSize()
   {
      return cellSize;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public void setOriginX(float originX)
   {
      this.originX = originX;
   }

   public void setOriginY(float originY)
   {
      this.originY = originY;
   }

   public void setCellSize(float cellSize)
   {
      this.cellSize = cellSize;
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
         float previousHeight = chunkHeights[i * cellsPerAxis + j];

         if (Math.abs(previousHeight - height) > 1E-4)
         {
            // If we updated a height value that didn't match what we already had, we say the chunk is dirty, and needs to be published.
            dirty = true;
            chunkHeights[i * cellsPerAxis + j] = height;
         }
      }
   }

   public void setHeight(int key, double height)
   {
      if (key >= 0 && key < chunkHeights.length)
      {
         chunkHeights[key] = (float) height;
      }
   }

   public static int generateHashForChunk(double xIndex, double yIndex)
   {
      int ix = (int) Math.floor(xIndex);
      int iy = (int) Math.floor(yIndex);

      return 13 * ix + 17 * iy;
   }

   public void setDirty(boolean dirty)
   {
      this.dirty = dirty;
   }

   public boolean isDirty()
   {
      return dirty;
   }

   public float[] getChunkHeights()
   {
      return chunkHeights;
   }

   public void setChunkHeights(float[] chunkHeights)
   {
      this.chunkHeights = chunkHeights;
   }

   public float getWidthInMeters()
   {
      return widthInMeters;
   }
}
