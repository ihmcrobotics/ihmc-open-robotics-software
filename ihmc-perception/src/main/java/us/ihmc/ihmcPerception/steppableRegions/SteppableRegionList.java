package us.ihmc.ihmcPerception.steppableRegions;

import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class SteppableRegionList
{
   private final List<SteppableRegion> regions;

   public SteppableRegionList()
   {
      regions = new ArrayList<>();
   }

   public SteppableRegionList(SteppableRegion... steppableRegions)
   {
      regions = new ArrayList<>();
      for (SteppableRegion steppableRegion : steppableRegions)
      {
         regions.add(steppableRegion);
      }
   }

   public SteppableRegionList(List<SteppableRegion> steppableRegions)
   {
      regions = new ArrayList<>(steppableRegions);
   }

   public SteppableRegionList(SteppableRegionList other)
   {
      this(other.getSteppableRegionsAsList());
   }

   public List<SteppableRegion> getSteppableRegionsAsList()
   {
      return regions;
   }
}
