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
   private PlanarRegionsList primaryPlanarRegions;
   private PlanarRegionsList fallbackPlanarRegions;
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

   private final ConvexPolygon2D primaryPlanarRegionModeledWorld = new ConvexPolygon2D();
   private final BoundingBox2D primaryPlanarRegionModeledBoundingBox = new BoundingBox2D();

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
      primaryPlanarRegions = null;
      fallbackPlanarRegions = null;
      fallbackHeightMap = null;
   }

   public void setPrimaryPlanarRegions(PlanarRegionsList primaryPlanarRegions)
   {
      this.primaryPlanarRegions = primaryPlanarRegions;
      environmentDataHolder.clear();

      computeHullOfRegions(primaryPlanarRegions, primaryPlanarRegionModeledWorld, primaryPlanarRegionModeledBoundingBox);
   }

   public void setFallbackHeightMap(HeightMapData fallbackHeightMap)
   {
      this.fallbackHeightMap = fallbackHeightMap;
      environmentDataHolder.clear();
   }

   public boolean flatGroundMode()
   {
      return !hasPrimaryPlanarRegions() && !hasFallbackPlanarRegions() && !hasFallbackHeightMap();
   }

   public boolean hasPrimaryPlanarRegions()
   {
      return primaryPlanarRegions != null && !primaryPlanarRegions.isEmpty();
   }

   public boolean hasFallbackPlanarRegions()
   {
      return fallbackPlanarRegions != null && !fallbackPlanarRegions.isEmpty();
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
         else if (hasPrimaryPlanarRegions() && isFootstepInRegionSet(primaryPlanarRegionModeledWorld, primaryPlanarRegionModeledBoundingBox))
         {
            environmentToUseForStep = EnvironmentToUse.PRIMARY_PLANAR_REGIONS;
         }
         else if (hasFallbackHeightMap())
         {
            environmentToUseForStep = EnvironmentToUse.HEIGHT_MAP;
         }
         else
         {
            environmentToUseForStep = EnvironmentToUse.PRIMARY_PLANAR_REGIONS;
         }

         environmentDataHolder.put(footstep, environmentToUseForStep);
         return environmentToUseForStep;
      }
   }

   public PlanarRegionsList getPlanarRegionsForFootstep(DiscreteFootstep footstep)
   {
      return getPlanarRegionsForFootstep(computeForFootstep(footstep));
   }

   public PlanarRegionsList getPlanarRegionsForFootstep(EnvironmentToUse environmentToUse)
   {
      if (!environmentToUse.isPlanarRegion())
         return null;

      if (environmentToUse == EnvironmentToUse.PRIMARY_PLANAR_REGIONS)
         return primaryPlanarRegions;
      else
         return fallbackPlanarRegions;
   }

   public PlanarRegionsList getPrimaryPlanarRegions()
   {
      return primaryPlanarRegions;
   }

   public HeightMapData getFallbackHeightMap()
   {
      return fallbackHeightMap;
   }

   private boolean isFootstepInRegionSet(ConvexPolygon2DReadOnly modeledHull, BoundingBox2DReadOnly regionModeledBoundingBox)
   {
      if (!footPolygon.getPolygonVerticesView().stream().allMatch(regionModeledBoundingBox::isInsideInclusive))
         return false;

      return footPolygon.getPolygonVerticesView().stream().allMatch(modeledHull::isPointInside);
   }

   private static void computeHullOfRegions(PlanarRegionsList regions, ConvexPolygon2DBasics modeledWorldToPack, BoundingBox2DBasics boundingBoxToPack)
   {
      modeledWorldToPack.clearAndUpdate();
      boundingBoxToPack.setToNaN();
      if (regions == null)
         return;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         ConvexPolygon2D convexHull = new ConvexPolygon2D(region.getConvexHull());
         convexHull.applyTransform(region.getTransformToWorld(), false);
         modeledWorldToPack.addVertices(convexHull);
         boundingBoxToPack.updateToIncludePoint(region.getBoundingBox3dInWorld().getMinX(), region.getBoundingBox3dInWorld().getMinY());
         boundingBoxToPack.updateToIncludePoint(region.getBoundingBox3dInWorld().getMaxX(), region.getBoundingBox3dInWorld().getMaxY());
      }
      modeledWorldToPack.update();
   }
}
