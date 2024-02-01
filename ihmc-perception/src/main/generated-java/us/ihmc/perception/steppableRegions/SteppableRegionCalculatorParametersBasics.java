package us.ihmc.perception.steppableRegions;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface SteppableRegionCalculatorParametersBasics extends SteppableRegionCalculatorParametersReadOnly, StoredPropertySetBasics
{
   default void setDistanceFromCliffBottoms(double distanceFromCliffBottoms)
   {
      set(SteppableRegionCalculatorParameters.distanceFromCliffBottoms, distanceFromCliffBottoms);
   }

   default void setDistanceFromCliffTops(double distanceFromCliffTops)
   {
      set(SteppableRegionCalculatorParameters.distanceFromCliffTops, distanceFromCliffTops);
   }

   default void setYawDiscretizations(int yawDiscretizations)
   {
      set(SteppableRegionCalculatorParameters.yawDiscretizations, yawDiscretizations);
   }

   default void setFootWidth(double footWidth)
   {
      set(SteppableRegionCalculatorParameters.footWidth, footWidth);
   }

   default void setFootLength(double footLength)
   {
      set(SteppableRegionCalculatorParameters.footLength, footLength);
   }

   default void setCliffStartHeightToAvoid(double cliffStartHeightToAvoid)
   {
      set(SteppableRegionCalculatorParameters.cliffStartHeightToAvoid, cliffStartHeightToAvoid);
   }

   default void setCliffEndHeightToAvoid(double cliffEndHeightToAvoid)
   {
      set(SteppableRegionCalculatorParameters.cliffEndHeightToAvoid, cliffEndHeightToAvoid);
   }

   default void setMinSupportAreaFraction(double minSupportAreaFraction)
   {
      set(SteppableRegionCalculatorParameters.minSupportAreaFraction, minSupportAreaFraction);
   }

   default void setMinSnapHeightThreshold(double minSnapHeightThreshold)
   {
      set(SteppableRegionCalculatorParameters.minSnapHeightThreshold, minSnapHeightThreshold);
   }

   default void setSnapHeightThresholdAtSearchEdge(double snapHeightThresholdAtSearchEdge)
   {
      set(SteppableRegionCalculatorParameters.snapHeightThresholdAtSearchEdge, snapHeightThresholdAtSearchEdge);
   }

   default void setInequalityActivationSlope(double inequalityActivationSlope)
   {
      set(SteppableRegionCalculatorParameters.inequalityActivationSlope, inequalityActivationSlope);
   }

   default void setMaxSearchDepthForRegions(int maxSearchDepthForRegions)
   {
      set(SteppableRegionCalculatorParameters.maxSearchDepthForRegions, maxSearchDepthForRegions);
   }

   default void setFractionOfCellToExpandSmallRegions(double fractionOfCellToExpandSmallRegions)
   {
      set(SteppableRegionCalculatorParameters.fractionOfCellToExpandSmallRegions, fractionOfCellToExpandSmallRegions);
   }

   default void setMaxInteriorPointsToInclude(int maxInteriorPointsToInclude)
   {
      set(SteppableRegionCalculatorParameters.maxInteriorPointsToInclude, maxInteriorPointsToInclude);
   }

   default void setMinCellsInARegion(int minCellsInARegion)
   {
      set(SteppableRegionCalculatorParameters.minCellsInARegion, minCellsInARegion);
   }

   default void setEdgeLengthThreshold(double edgeLengthThreshold)
   {
      set(SteppableRegionCalculatorParameters.edgeLengthThreshold, edgeLengthThreshold);
   }
}
