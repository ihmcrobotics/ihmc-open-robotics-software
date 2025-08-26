package us.ihmc.footstepPlanning.steppableRegions;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.HeightMapParameters;

public class TerrainMapDataTest
{
   int cellsPerAxis = 100; // 2 m x 2 m
   private final HeightMapParameters parameters = new HeightMapParameters();
   private final TerrainMapData terrainMapData = new TerrainMapData(cellsPerAxis, parameters.getCellSize(), parameters.getTerrainWidthInMeters());

   @Test
   public void testTerrainMapSurfaceNormals()
   {
      // set the middle 20x20 cells to ramp (0.4 m x 0.4 m)
      for (int i = cellsPerAxis / 2 - 10; i < cellsPerAxis / 2 + 10; i++)
      {
         for (int j = cellsPerAxis / 2 - 10; j < cellsPerAxis / 2 + 10; j++)
         {
            terrainMapData.setHeightFloatLocal(i * (1 / 50.0f), i, j);
         }
      }

      //PerceptionDebugTools.printMat("Height Map", terrainMapData.getHeightMap(), 1);

      LogTools.info("Normal: {}", TerrainMapTools.computeSurfaceNormalInWorld(terrainMapData, 0.3f, 0.3f, 1));
   }
}
