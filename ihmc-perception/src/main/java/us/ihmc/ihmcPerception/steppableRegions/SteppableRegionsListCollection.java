package us.ihmc.ihmcPerception.steppableRegions;

import java.util.ArrayList;
import java.util.List;

public class SteppableRegionListCollection
{
   private final List<SteppableRegionsList> steppableRegions;

   public SteppableRegionListCollection()
   {
      steppableRegions = new ArrayList<>();
   }

   public SteppableRegionListCollection(SteppableRegionsList... steppableRegions)
   {
      // TODO check the different yaws
      this.steppableRegions = new ArrayList<>();
      for (SteppableRegionsList steppableRegion : steppableRegions)
      {
         this.steppableRegions.add(steppableRegion);
      }
   }

   public SteppableRegionListCollection(List<SteppableRegionsList> steppableRegions)
   {
      // TODO check the different yaws
      this.steppableRegions = new ArrayList<>(steppableRegions);
   }

   public SteppableRegionListCollection(SteppableRegionListCollection other)
   {
      this(other.getSteppableRegions());
   }

   public List<SteppableRegionsList> getSteppableRegions()
   {
      return steppableRegions;
   }
}
