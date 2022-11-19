package us.ihmc.perception.mapping;

import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionFilteredMap
{
   private PlanarRegionsList slamMap;

   boolean initialized = false;
   boolean modified = false;

   public PlanarRegionFilteredMap()
   {
   }

   public void submitRegions(PlanarRegionsList regions)
   {
      if(!initialized)
      {
         slamMap.addPlanarRegionsList(regions);
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

