package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ConcaveHullFactoryParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getEdgeLengthThreshold()
   {
      return get(edgeLengthThreshold);
   }

   default boolean getRemoveAllTrianglesWithTwoBorderEdges()
   {
      return get(removeAllTrianglesWithTwoBorderEdges);
   }

   default boolean getAllowSplittingConcaveHull()
   {
      return get(allowSplittingConcaveHull);
   }

   default int getMaxNumberOfIterations()
   {
      return get(maxNumberOfIterations);
   }

   default double getTriangulationTolerance()
   {
      return get(triangulationTolerance);
   }
}
