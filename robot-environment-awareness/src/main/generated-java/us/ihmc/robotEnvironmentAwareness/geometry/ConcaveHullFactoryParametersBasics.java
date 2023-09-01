package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ConcaveHullFactoryParametersBasics extends ConcaveHullFactoryParametersReadOnly, StoredPropertySetBasics
{
   default void setEdgeLengthThreshold(double edgeLengthThreshold)
   {
      set(ConcaveHullFactoryParameters.edgeLengthThreshold, edgeLengthThreshold);
   }

   default void setRemoveAllTrianglesWithTwoBorderEdges(boolean removeAllTrianglesWithTwoBorderEdges)
   {
      set(ConcaveHullFactoryParameters.removeAllTrianglesWithTwoBorderEdges, removeAllTrianglesWithTwoBorderEdges);
   }

   default void setAllowSplittingConcaveHull(boolean allowSplittingConcaveHull)
   {
      set(ConcaveHullFactoryParameters.allowSplittingConcaveHull, allowSplittingConcaveHull);
   }

   default void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      set(ConcaveHullFactoryParameters.maxNumberOfIterations, maxNumberOfIterations);
   }

   default void setTriangulationTolerance(double triangulationTolerance)
   {
      set(ConcaveHullFactoryParameters.triangulationTolerance, triangulationTolerance);
   }
}
