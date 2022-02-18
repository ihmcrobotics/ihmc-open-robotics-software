package us.ihmc.robotEnvironmentAwareness.communication.converters;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface ScanRegionFilter
{
   boolean test(int index, PlanarRegion point);

   public static ScanRegionFilter combine(ScanRegionFilter filterA, ScanRegionFilter filterB)
   {
      return (index, point) -> filterA.test(index, point) && filterB.test(index, point);
   }
}