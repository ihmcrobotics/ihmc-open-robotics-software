package us.ihmc.perception.gpuHeightMap.worldModel;

import com.esotericsoftware.kryo.util.IntMap;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.nio.ShortBuffer;
import java.util.Collection;
import java.util.HashSet;

public class ChunkedMap
{
   private final IntMap<Chunk> chunksHashMap = new IntMap<>();
   private final HashSet<Chunk> chunks = new HashSet<>();

   public ChunkedMap()
   {
   }

   public void addHeightMap(Mat heightMap, Point3DReadOnly heightMapCenter, double gridSize, double resolution, double heightOffset, double scalingFactor)
   {
      chunks.clear();

      int centerIndex = HeightMapTools.computeCenterIndex(gridSize, resolution);

      int centerIndexOfIncomingHeightMap = HeightMapTools.computeCenterIndex(gridSize, resolution);
      int cellsPerAxisOfIncomingHeightMap = 2 * centerIndexOfIncomingHeightMap + 1;
      int totalCellsOfIncomingHeightMap = cellsPerAxisOfIncomingHeightMap * cellsPerAxisOfIncomingHeightMap;
      // This is done for speed optimization
      short[] heightsArray = new short[totalCellsOfIncomingHeightMap];

      ShortBuffer shortBuffer = heightMap.createBuffer();
      shortBuffer.get(heightsArray);

      for (int i = 0; i < heightMap.rows(); i++)
      {
         for (int j = 0; j < heightMap.cols(); j++)
         {
            double XCord = HeightMapTools.indexToCoordinate(i, heightMapCenter.getX(), resolution, centerIndex);
            double YCord = HeightMapTools.indexToCoordinate(j, heightMapCenter.getY(), resolution, centerIndex);

            Chunk chunk = getOrCreateChunk(XCord, YCord, Chunk.CHUNK_WIDTH, resolution, heightOffset, scalingFactor);

            int index = i * cellsPerAxisOfIncomingHeightMap + j;
            short height = heightsArray[index];

            chunk.setHeightAt(XCord, YCord, height, resolution);

            chunks.add(chunk);
         }
      }
   }

   public Collection<Chunk> getChunks()
   {
      return chunks;
   }

   public Chunk getOrCreateChunk(double xCoordinate,
                                 double yCoordinate,
                                 double chunkSizeInMeters,
                                 double chunkResolution,
                                 double heightOffset,
                                 double scalingFactor)
   {
      int cellsPerAxis = (int) Math.round(chunkSizeInMeters / chunkResolution);

      int worldCellX = (int) Math.floor(xCoordinate / chunkResolution);
      int worldCellY = (int) Math.floor(yCoordinate / chunkResolution);

      int chunkIndexX = (int) Math.floor((double) worldCellX / cellsPerAxis);
      int chunkIndexY = (int) Math.floor((double) worldCellY / cellsPerAxis);

      /*
       * Note: This is very important, the chunk maps need to align on the same grid layout as the height map
       * We do this because the center of cell [0,0] for the height map is world is (0.0, 0.0).
       * And so we need the center of cell [0,0] for the chunk map to also be at world (0.0, 0.0)
       * So the (0.5 * resolution) shifts the origin to be aligned with the height map
       */
      double chunkOriginX = chunkIndexX * chunkSizeInMeters - 0.5 * chunkResolution;
      double chunkOriginY = chunkIndexY * chunkSizeInMeters - 0.5 * chunkResolution;

      int hash = Chunk.generateHashForChunk(chunkOriginX, chunkOriginY);

      Chunk chunk = chunksHashMap.get(hash);
      if (chunk == null)
      {
         chunk = new Chunk(chunkResolution, chunkOriginX, chunkOriginY, heightOffset, scalingFactor);
         chunksHashMap.put(hash, chunk);
      }

      return chunk;
   }
}

