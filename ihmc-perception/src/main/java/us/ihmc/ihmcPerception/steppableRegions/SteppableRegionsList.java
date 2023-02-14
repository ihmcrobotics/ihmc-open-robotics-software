package us.ihmc.ihmcPerception.steppableRegions;

import java.util.ArrayList;
import java.util.List;

public class SteppableRegionsList
{
   private final double footYaw;
   private final double footLength;
   private final double footWidth;

   private final List<SteppableRegion> regions;

   public SteppableRegionsList(double footYaw, double footLength, double footWidth)
   {
      this(footYaw, footLength, footWidth, new ArrayList<>());
   }

   public SteppableRegionsList(double footYaw, double footLength, double footWidth, SteppableRegion... steppableRegions)
   {
      this.footYaw = footYaw;
      this.footLength = footLength;
      this.footWidth = footWidth;
      regions = new ArrayList<>();
      for (SteppableRegion steppableRegion : steppableRegions)
      {
         regions.add(steppableRegion);
      }
   }

   public SteppableRegionsList(double footYaw, double footLength, double footWidth, List<SteppableRegion> steppableRegions)
   {
      this.footYaw = footYaw;
      this.footLength = footLength;
      this.footWidth = footWidth;
      regions = new ArrayList<>(steppableRegions);
   }

   public SteppableRegionsList(SteppableRegionsList other)
   {
      this(other.getFootYaw(), other.getFootLength(), other.getFootWidth(), other.getSteppableRegionsAsList());
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

   public double getFootLength()
   {
      return footLength;
   }

   public double getFootWidth()
   {
      return footWidth;
   }
}
