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

   public void addHeightMap(Mat heightMap, Point3DReadOnly heightMapCenter, double gridSize, double gridResolution)
   {
      chunks.clear();

      int centerIndex = HeightMapTools.computeCenterIndex(gridSize, gridResolution);

      int centerIndexOfIncomingHeightMap = HeightMapTools.computeCenterIndex(gridSize, gridResolution);
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
            double XCord = HeightMapTools.indexToCoordinate(i, heightMapCenter.getX(), gridResolution, centerIndex);
            double YCord = HeightMapTools.indexToCoordinate(j, heightMapCenter.getY(), gridResolution, centerIndex);

            Chunk chunk = getOrCreateChunk(XCord, YCord, Chunk.LATTICE_WIDTH, gridResolution);

            int index = i * cellsPerAxisOfIncomingHeightMap + j;
            short height = heightsArray[index];

            int centerIndexOfChunk = HeightMapTools.computeCenterIndex(Chunk.LATTICE_WIDTH, gridResolution);
            chunk.setHeightAt(XCord, YCord, height, gridResolution, centerIndexOfChunk);

            chunks.add(chunk);
         }
      }
   }

   public Collection<Chunk> getChunks()
   {
      return chunks;
   }

   public Chunk getOrCreateChunk(double xCoordinate, double yCoordinate, double chunkSizeInMeters, double chunkResolution)
   {
      int cellsPerAxis = (int) Math.round(chunkSizeInMeters / chunkResolution);

      int worldCellX = (int) Math.floor(xCoordinate / chunkResolution);
      int worldCellY = (int) Math.floor(yCoordinate / chunkResolution);

      int chunkIndexX = (int) Math.floor((double) worldCellX / cellsPerAxis);
      int chunkIndexY = (int) Math.floor((double) worldCellY / cellsPerAxis);

      double chunkCenterX = (chunkIndexX + 0.5) * chunkSizeInMeters;
      double chunkCenterY = (chunkIndexY + 0.5) * chunkSizeInMeters;
      int hash = Chunk.generateHashForChunk(chunkCenterX, chunkCenterY);

      Chunk chunk = chunksHashMap.get(hash);
      if (chunk == null)
      {
         chunk = new Chunk(chunkResolution, chunkCenterX, chunkCenterY);
         chunksHashMap.put(hash, chunk);
      }

      return chunk;
   }
}

