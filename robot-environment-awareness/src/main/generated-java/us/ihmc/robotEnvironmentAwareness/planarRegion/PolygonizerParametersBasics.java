package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PolygonizerParametersBasics extends PolygonizerParametersReadOnly, StoredPropertySetBasics
{
   /**
    * Threshold used when creating a new concave hull with {@link
    * us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory}. Uses the
    * Duckham and al. (2008) algorithm defined in the paper entitled "Efficient
    * generation of simple polygons for characterizing the shape of a set of points in
    * the plane".
    */
   default void setConcaveHullThreshold(double concaveHullThreshold)
   {
      set(PolygonizerParameters.concaveHullThreshold, concaveHullThreshold);
   }

   /**
    * The minimum number of nodes required for a {@link
    * us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData}
    * to be polygonized.
    */
   default void setMinNumberOfNodes(int minNumberOfNodes)
   {
      set(PolygonizerParameters.minNumberOfNodes, minNumberOfNodes);
   }

   /**
    * Filter parameter on the concave hull of a region. Used to removed vertices
    * describing shallow angle.
    */
   default void setShallowAngleThreshold(double shallowAngleThreshold)
   {
      set(PolygonizerParameters.shallowAngleThreshold, shallowAngleThreshold);
   }

   /**
    * Filter parameter on the concave hull of a region. Used to removed vertices that
    * create peaks.
    */
   default void setPeakAngleThreshold(double peakAngleThreshold)
   {
      set(PolygonizerParameters.peakAngleThreshold, peakAngleThreshold);
   }

   /**
    * Filter parameter on the concave hull of a region. Used to removed short edges.
    */
   default void setLengthThreshold(double lengthThreshold)
   {
      set(PolygonizerParameters.lengthThreshold, lengthThreshold);
   }

   /**
    * Threshold used for decomposing the concave hull into convex polygons. Describes
    * the maximum depth of a concavity before the concave hull gets split in 2.
    */
   default void setDepthThreshold(double depthThreshold)
   {
      set(PolygonizerParameters.depthThreshold, depthThreshold);
   }

   /**
    * Filter for splitting concave hulls at any narrow passage which width is less
    * than 2 * lengthThreshold.
    */
   default void setCutNarrowPassage(boolean cutNarrowPassage)
   {
      set(PolygonizerParameters.cutNarrowPassage, cutNarrowPassage);
   }
}
