package us.ihmc.perception.steppableRegions;

import java.util.ArrayList;
import java.util.List;

public class SteppableRegionsList
{
   private final double footYaw;

   private final List<SteppableRegion> regions;

   public SteppableRegionsList(double footYaw)
   {
      this(footYaw, new ArrayList<>());
   }

   public SteppableRegionsList(double footYaw, SteppableRegion... steppableRegions)
   {
      this.footYaw = footYaw;
      regions = new ArrayList<>();
      for (SteppableRegion steppableRegion : steppableRegions)
      {
         regions.add(steppableRegion);
      }
   }

   public SteppableRegionsList(double footYaw,  List<SteppableRegion> steppableRegions)
   {
      this.footYaw = footYaw;
      regions = new ArrayList<>(steppableRegions);
   }

   public SteppableRegionsList(SteppableRegionsList other)
   {
      this(other.getFootYaw(), other.getSteppableRegionsAsList());
   }

   public List<SteppableRegion> getSteppableRegionsAsList()
   {
      return regions;
   }

   public int getNumberOfSteppableRegions()
   {
      return regions.size();
   }

   public SteppableRegion getSteppableRegion(int i)
   {
      return regions.get(i);
   }

   public double getFootYaw()
   {
      return footYaw;
   }

   @Override
   public String toString()
   {
      return "List with " + regions.size() + " regions.";
   }
}
