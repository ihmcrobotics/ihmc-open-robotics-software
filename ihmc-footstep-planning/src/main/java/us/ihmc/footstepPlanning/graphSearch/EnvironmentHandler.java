package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.steppableRegions.TerrainMapData;
import us.ihmc.perception.heightMap.HeightMapData;

public class EnvironmentHandler
{
   private HeightMapData heightMap;
   private TerrainMapData terrainMapData;

   public void clear()
   {
      heightMap = null;
      terrainMapData = null;
   }

   public void setHeightMapData(HeightMapData heightMap)
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

   public HeightMapData getHeightMapData()
   {
      return heightMap;
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }
}
