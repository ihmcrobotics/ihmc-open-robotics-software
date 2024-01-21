package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class FootstepPlannerEnvironmentHandler
{
   private HeightMapData heightMap;
   private TerrainMapData terrainMapData;

   public void reset()
   {
      heightMap = null;
      terrainMapData = null;
   }

   public void setHeightMap(HeightMapData heightMap)
   {
      this.heightMap = heightMap;
   }

   public void setTerrainMapData(TerrainMapData terrainMapData)
   {
      this.terrainMapData = terrainMapData;
   }

   public boolean flatGroundMode()
   {
      return !hasHeightMap() && !hasTerrainMapData();
   }

   public boolean hasHeightMap()
   {
      return heightMap != null && !heightMap.isEmpty();
   }

   public boolean hasTerrainMapData()
   {
      // TODO any more going in here?
      return terrainMapData != null;
   }

   public HeightMapData getHeightMap()
   {
      return heightMap;
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }
}
