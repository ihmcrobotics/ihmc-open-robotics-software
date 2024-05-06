package us.ihmc.perception.gpuHeightMap;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.PerceptionDebugTools;

public class TerrainMapDataTest
{
   int size = 100; // 2 m x 2 m
   private TerrainMapData terrainMapData = new TerrainMapData(size, size);

   @Test
   public void testTerrainMapSurfaceNormals()
   {
      // set the middle 20x20 cells to ramp (0.4 m x 0.4 m)
      for (int i = size / 2 - 10; i < size / 2 + 10; i++)
      {
         for (int j = size / 2 - 10; j < size / 2 + 10; j++)
         {
            terrainMapData.setHeightLocal(i * (1 / 50.0f), i,j);
         }
      }

      //PerceptionDebugTools.printMat("Height Map", terrainMapData.getHeightMap(), 1);

      LogTools.info("Normal: {}", terrainMapData.computeSurfaceNormalInWorld(0.3f, 0.3f));
   }
}
