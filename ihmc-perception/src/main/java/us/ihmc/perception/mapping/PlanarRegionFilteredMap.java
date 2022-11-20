package us.ihmc.perception.mapping;

import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;

public class PlanarRegionFilteredMap
{
   private PlanarRegionsList slamMap;

   boolean initialized = false;
   boolean modified = false;

   public PlanarRegionFilteredMap()
   {
      slamMap = new PlanarRegionsList();
   }

   public void submitRegions(PlanarRegionsList regions)
   {
      if(!initialized)
      {
         slamMap.addPlanarRegionsList(regions);
      }
      else
      {
         HashMap<Integer, Integer> matches = PlanarRegionSLAMTools.findPlanarRegionMatches(slamMap, regions, 0.5f, 0.7f);
      }

   }

   public PlanarRegionsList getMapRegions()
   {
      return slamMap;
   }

   public boolean isModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }
}

