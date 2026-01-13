package us.ihmc.perception.gpuMapping.worldModel;

import com.esotericsoftware.kryo.util.IntMap;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.gpuMapping.HeightMapParameters;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import java.nio.FloatBuffer;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Deque;
import java.util.HashSet;

/**
 * This class represents a bunch of {@link Chunk}'s that are used to create the {@link ChunkedMapManager}.
 * As an overview, the heightmap is somewhere in the world. We take that in and map the data to a bunch of {@link Chunk}'s that represent our
 * {@link ChunkedMapManager}. The {@link ChunkedMapManager} is aligned on our world grid. The cells of the height map align with the cells of the
 * {@link ChunkedMapManager} so there
 * aren't any cases where the data could land on the edge of two cells.
 * Once we've added the height map into {@link Chunk}'s we are ready to use this somewhere else.
 */
public class ChunkedMapManager
{
   private static final int MAX_CHUNKS_TO_STORE = 100;
   private final HeightMapParameters heightMapParameters;

   private final IntMap<Chunk> chunksHashMap = new IntMap<>();
   private final HashSet<Chunk> chunks = new HashSet<>();
   private final Deque<Integer> queueOfChunks = new ArrayDeque<>();
   private final ROS2Publisher<ChunkMessage> chunkMessagePublisher;
   private final ChunkMessage chunkMessage;

   public ChunkedMapManager(ROS2Node ros2Node, HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;
      chunkMessagePublisher = ros2Node.createPublisher(PerceptionAPI.CHUNK);
      chunkMessage = new ChunkMessage();
   }

   public void updateAndPublish(Mat latestHeightMap, Point3DReadOnly heightMapCenterPoint)
   {
      if (heightMapParameters.getEnableChunkedMap())
      {
         addHeightMap(latestHeightMap, heightMapCenterPoint, heightMapParameters.getWidthInMeters(), (float) heightMapParameters.getCellSize());
         publishChunkedMap(chunkMessagePublisher);
      }
   }

   public void addHeightMap(Mat heightMap, Point3DReadOnly heightMapCenter, double gridSize, float resolution)
   {
      chunks.clear();

      int centerIndexOfIncomingHeightMap = HeightMapTools.computeCenterIndex(gridSize, resolution);
      int cellsPerAxisOfIncomingHeightMap = 2 * centerIndexOfIncomingHeightMap + 1;
      int totalCellsOfIncomingHeightMap = cellsPerAxisOfIncomingHeightMap * cellsPerAxisOfIncomingHeightMap;
      // This is done for speed optimization
      float[] heightsArray = new float[totalCellsOfIncomingHeightMap];

      FloatBuffer floatBuffer = heightMap.createBuffer();
      floatBuffer.get(heightsArray);

      for (int i = 0; i < heightMap.rows(); i++)
      {
         for (int j = 0; j < heightMap.cols(); j++)
         {
            double XCord = HeightMapTools.indexToCoordinate(i, heightMapCenter.getX(), resolution, centerIndexOfIncomingHeightMap);
            double YCord = HeightMapTools.indexToCoordinate(j, heightMapCenter.getY(), resolution, centerIndexOfIncomingHeightMap);
            Chunk chunk = getOrCreateChunk(XCord, YCord, Chunk.CHUNK_WIDTH, resolution);

            int index = i * cellsPerAxisOfIncomingHeightMap + j;
            float height = heightsArray[index];

            chunk.setHeightAt(XCord, YCord, height, resolution);

            chunks.add(chunk);
         }
      }
   }

   /**
    * We don't want to create new {@link Chunk}'s if we have already made on.
    * So we pull from our map of {@link Chunk}'s to check, if we don't have one, we create a new one.
    * Which is why we need to pass in so many parameters.
    */
   private Chunk getOrCreateChunk(double xCoordinate, double yCoordinate, float chunkSizeInMeters, float chunkResolution)
   {
      int cellsPerAxis = Math.round(chunkSizeInMeters / chunkResolution);

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
      float chunkOriginX = chunkIndexX * chunkSizeInMeters - 0.5f * chunkResolution;
      float chunkOriginY = chunkIndexY * chunkSizeInMeters - 0.5f * chunkResolution;

      int hash = Chunk.generateHashForChunk(chunkOriginX, chunkOriginY);

      Chunk chunk = chunksHashMap.get(hash);
      if (chunk == null)
      {
         chunk = new Chunk(chunkOriginX, chunkOriginY, chunkResolution, cellsPerAxis);
         chunksHashMap.put(hash, chunk);
         queueOfChunks.addLast(hash);

         // Resources aren't free, at some point there is a limit on how much we can store, time to see if we should remove any
         if (chunksHashMap.size > MAX_CHUNKS_TO_STORE)
         {
            Integer oldestHash = queueOfChunks.pollFirst();
            if (oldestHash != null)
            {
               chunksHashMap.remove(oldestHash);
            }
         }
      }

      return chunk;
   }

   private void publishChunkedMap(ROS2Publisher<ChunkMessage> publisher)
   {
      Collection<Chunk> chunks = getChunks();
      for (Chunk chunk : chunks)
      {
         // If the chunk doesn't have any new height values, we don't need to publish it again
         if (!chunk.isDirty())
            continue;

         chunkMessage.setHashCodeOfChunk(chunk.hashCode());

         ChunkMessageTools.toMessage(chunk, chunkMessage);
         publisher.publish(chunkMessage);
         // After it's been published, we reset the dirty value so we don't keep publishing it even
         // if it hasn't changed
         chunk.setDirty(false);
      }
   }

   public Collection<Chunk> getChunks()
   {
      return chunks;
   }

   public void destroy()
   {
      chunkMessagePublisher.remove();
      chunks.clear();
      chunksHashMap.clear();
      queueOfChunks.clear();
   }
}

