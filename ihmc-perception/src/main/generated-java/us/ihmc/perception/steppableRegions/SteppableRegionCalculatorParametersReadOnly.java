package us.ihmc.perception.steppableRegions;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface SteppableRegionCalculatorParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getDistanceFromCliffBottoms()
   {
      return get(distanceFromCliffBottoms);
   }

   default double getDistanceFromCliffTops()
   {
      return get(distanceFromCliffTops);
   }

   default int getYawDiscretizations()
   {
      return get(yawDiscretizations);
   }

   default double getFootWidth()
   {
      return get(footWidth);
   }

   default double getFootLength()
   {
      return get(footLength);
   }

   default double getCliffStartHeightToAvoid()
   {
      return get(cliffStartHeightToAvoid);
   }

   default double getCliffEndHeightToAvoid()
   {
      return get(cliffEndHeightToAvoid);
   }

   default int getMaxSearchDepthForRegions()
   {
      return get(maxSearchDepthForRegions);
   }

   default double getFractionOfCellToExpandSmallRegions()
   {
      return get(fractionOfCellToExpandSmallRegions);
   }

   default int getMaxInteriorPointsToInclude()
   {
      return get(maxInteriorPointsToInclude);
   }

   default int getMinCellsInARegion()
   {
      return get(minCellsInARegion);
   }

   default double getEdgeLengthThreshold()
   {
      return get(edgeLengthThreshold);
   }
}
