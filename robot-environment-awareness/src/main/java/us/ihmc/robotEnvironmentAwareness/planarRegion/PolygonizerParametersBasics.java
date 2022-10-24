package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.tools.property.StoredPropertySetBasics;

public interface PolygonizerParametersBasics extends PolygonizerParametersReadOnly, StoredPropertySetBasics
{
   default void setConcaveHullThreshold(double concaveHullThreshold)
   {
      set(PolygonizerParameters.concaveHullThreshold, concaveHullThreshold);
   }

   default void setMinNumberOfNodes(int minNumberOfNodes)
   {
      set(PolygonizerParameters.minNumberOfNodes, minNumberOfNodes);
   }

   default void setShallowAngleThreshold(double shallowAngleThreshold)
   {
      set(PolygonizerParameters.shallowAngleThreshold, shallowAngleThreshold);
   }

   default void setPeakAngleThreshold(double peakAngleThreshold)
   {
      set(PolygonizerParameters.peakAngleThreshold, peakAngleThreshold);
   }

   default void setLengthThreshold(double lengthThreshold)
   {
      set(PolygonizerParameters.lengthThreshold, lengthThreshold);
   }

   default void setDepthThreshold(double depthThreshold)
   {
      set(PolygonizerParameters.depthThreshold, depthThreshold);
   }

   default void setCutNarrowPassage(boolean cutNarrowPassage)
   {
      set(PolygonizerParameters.cutNarrowPassage, cutNarrowPassage);
   }
}
