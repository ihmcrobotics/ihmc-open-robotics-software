package us.ihmc.ihmcPerception.steppableRegions;

import java.util.ArrayList;
import java.util.List;

public class SteppableRegionListCollection
{
   private final List<SteppableRegionList> steppableRegions;

   public SteppableRegionListCollection()
   {
      steppableRegions = new ArrayList<>();
   }

   public SteppableRegionListCollection(SteppableRegionList... steppableRegions)
   {
      // TODO check the different yaws
      this.steppableRegions = new ArrayList<>();
      for (SteppableRegionList steppableRegion : steppableRegions)
      {
         this.steppableRegions.add(steppableRegion);
      }
   }

   public SteppableRegionListCollection(List<SteppableRegionList> steppableRegions)
   {
      // TODO check the different yaws
      this.steppableRegions = new ArrayList<>(steppableRegions);
   }

   public SteppableRegionListCollection(SteppableRegionListCollection other)
   {
      this(other.getSteppableRegions());
   }

   public List<SteppableRegionList> getSteppableRegions()
   {
      return steppableRegions;
   }
}
