package us.ihmc.perception.geometry;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.geometry.ConcaveHullFactoryParameters.*;

public interface ConcaveHullFactoryParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getEdgeLengthThreshold()
   {
      return get(edgeLengthThreshold);
   }

   default boolean doRemoveAllTrianglesWithTwoBorderEdges()
   {
      return get(removeAllTrianglesWithTwoBorderEdges);
   }

   default boolean isSplittingConcaveHullAllowed()
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
