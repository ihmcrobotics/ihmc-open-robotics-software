package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.*;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.HashMap;

public class FootstepPlannerEnvironmentHandler
{
   private HeightMapData fallbackHeightMap;

   public enum EnvironmentToUse
   {
      FLAT_GROUND(false),
      PRIMARY_PLANAR_REGIONS(true),
      HEIGHT_MAP(false);

      private final boolean isPlanarRegion;

      private EnvironmentToUse(boolean isPlanarRegion)
      {
         this.isPlanarRegion = isPlanarRegion;
      }

      public boolean isPlanarRegion()
      {
         return isPlanarRegion;
      }
   }

   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   private final HashMap<DiscreteFootstep, EnvironmentToUse> environmentDataHolder = new HashMap<>();

   public FootstepPlannerEnvironmentHandler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
   }

   public void reset()
   {
      environmentDataHolder.clear();
      fallbackHeightMap = null;
   }

   public void setFallbackHeightMap(HeightMapData fallbackHeightMap)
   {
      this.fallbackHeightMap = fallbackHeightMap;
      environmentDataHolder.clear();
   }

   public boolean flatGroundMode()
   {
      return !hasFallbackHeightMap();
   }

   public boolean hasFallbackHeightMap()
   {
      return fallbackHeightMap != null && !fallbackHeightMap.isEmpty();
   }

   public EnvironmentToUse computeForFootstep(DiscreteFootstep footstep)
   {
      if (environmentDataHolder.containsKey(footstep))
      {
         return environmentDataHolder.get(footstep);
      }
      else
      {
         DiscreteFootstepTools.getFootPolygon(footstep, footPolygonsInSoleFrame.get(footstep.getRobotSide()), footPolygon);

         EnvironmentToUse environmentToUseForStep;
         if (flatGroundMode())
         {
            environmentToUseForStep = EnvironmentToUse.FLAT_GROUND;
         }
         else if (hasFallbackHeightMap())
         {
            environmentToUseForStep = EnvironmentToUse.HEIGHT_MAP;
         }
         else
         {
            environmentToUseForStep = EnvironmentToUse.FLAT_GROUND;
         }

         environmentDataHolder.put(footstep, environmentToUseForStep);
         return environmentToUseForStep;
      }
   }

   public HeightMapData getFallbackHeightMap()
   {
      return fallbackHeightMap;
   }
}
