package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.HashMap;

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
