package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters.*;

public interface PolygonizerParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getConcaveHullThreshold()
   {
      return get(concaveHullThreshold);
   }

   default int getMinNumberOfNodes()
   {
      return get(minNumberOfNodes);
   }

   default double getShallowAngleThreshold()
   {
      return get(shallowAngleThreshold);
   }

   default double getPeakAngleThreshold()
   {
      return get(peakAngleThreshold);
   }

   default double getLengthThreshold()
   {
      return get(lengthThreshold);
   }

   default double getDepthThreshold()
   {
      return get(depthThreshold);
   }

   default boolean getCutNarrowPassage()
   {
      return get(cutNarrowPassage);
   }
}
