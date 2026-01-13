package us.ihmc.perception.heightmap;

import org.apache.log4j.Hierarchy;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.gpuMapping.HeightMapParameters;
import us.ihmc.perception.gpuMapping.worldModel.Chunk;
import us.ihmc.perception.gpuMapping.worldModel.ChunkTools;
import us.ihmc.perception.gpuMapping.worldModel.ChunkedMapManager;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

import java.util.Collection;

public class ChunkedMapManagerTest
{
   @Test
   public void testChunksGetHeightsCorrectly()
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build("test_node");
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      heightMapParameters.setCellSize(0.02);
      heightMapParameters.setWidthInMeters(1.0);
      ChunkedMapManager chunkedMapManager = new ChunkedMapManager(ros2Node, heightMapParameters);

      // This test is setup so that all the chunks will be filled will values of100
      int cellsPerAxis = 51;
      Mat heightMapMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            heightMapMat.ptr(i, j).putFloat(100.0f);
         }
      }

      chunkedMapManager.addHeightMap(heightMapMat, new Point3D(2.0, 2.0, 0.0), heightMapParameters.getWidthInMeters(), heightMapParameters.getCellSize());

      Collection<Chunk> chunks = chunkedMapManager.getChunks();
      // Because the cells per axis is 101 per side, we expect to get 3 chunks for that, on each axis.

      for (Chunk chunk : chunks)
      {
         Mat what = new Mat(chunk.getCellsPerAxis(), chunk.getCellsPerAxis(), opencv_core.CV_32FC1);
         ChunkTools.convertToMat(what, chunk);
         PerceptionDebugTools.printMat("s", what, 5);
//         float[] chunkHeights = chunk.getChunkHeights();
//         for (int i = 0; i < chunkHeights.length; i++)
//         {
//            assertEquals(chunkHeights[i], 100.0);
//         }
      }
   }
}
