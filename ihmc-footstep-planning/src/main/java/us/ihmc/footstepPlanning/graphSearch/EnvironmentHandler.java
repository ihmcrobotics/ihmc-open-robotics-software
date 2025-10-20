package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.perception.gpuMapping.TerrainMapData;

public class EnvironmentHandler
{
   private TerrainMapData terrainMapData;

   public void clear()
   {
      terrainMapData = null;
   }

   public void setTerrainMapData(TerrainMapData terrainMapData)
   {
      this.terrainMapData = terrainMapData;
   }

   public boolean flatGroundMode()
   {
      return !hasTerrainMapData();
   }

   public boolean hasTerrainMapData()
   {
      return terrainMapData != null;
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }
}
