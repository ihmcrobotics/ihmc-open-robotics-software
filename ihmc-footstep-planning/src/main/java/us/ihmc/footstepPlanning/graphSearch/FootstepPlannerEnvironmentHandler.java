package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class FootstepPlannerEnvironmentHandler
{
   private HeightMapData heightMap;

   public void reset()
   {
      heightMap = null;
   }

   public void setHeightMap(HeightMapData heightMap)
   {
      this.heightMap = heightMap;
   }

   public boolean flatGroundMode()
   {
      return !hasHeightMap();
   }

   public boolean hasHeightMap()
   {
      return heightMap != null && !heightMap.isEmpty();
   }

   public HeightMapData getHeightMap()
   {
      return heightMap;
   }
}
