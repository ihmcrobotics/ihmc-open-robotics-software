package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PolygonizerParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Threshold used when creating a new concave hull with {@link
    * us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory}. Uses the
    * Duckham and al. (2008) algorithm defined in the paper entitled "Efficient
    * generation of simple polygons for characterizing the shape of a set of points in
    * the plane".
    */
   default double getConcaveHullThreshold()
   {
      return get(concaveHullThreshold);
   }

   /**
    * The minimum number of nodes required for a {@link
    * us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData}
    * to be polygonized.
    */
   default int getMinNumberOfNodes()
   {
      return get(minNumberOfNodes);
   }

   /**
    * Filter parameter on the concave hull of a region. Used to removed vertices
    * describing shallow angle.
    */
   default double getShallowAngleThreshold()
   {
      return get(shallowAngleThreshold);
   }

   /**
    * Filter parameter on the concave hull of a region. Used to removed vertices that
    * create peaks.
    */
   default double getPeakAngleThreshold()
   {
      return get(peakAngleThreshold);
   }

   /**
    * Filter parameter on the concave hull of a region. Used to removed short edges.
    */
   default double getLengthThreshold()
   {
      return get(lengthThreshold);
   }

   /**
    * Threshold used for decomposing the concave hull into convex polygons. Describes
    * the maximum depth of a concavity before the concave hull gets split in 2.
    */
   default double getDepthThreshold()
   {
      return get(depthThreshold);
   }

   /**
    * Filter for splitting concave hulls at any narrow passage which width is less
    * than 2 * lengthThreshold.
    */
   default boolean getCutNarrowPassage()
   {
      return get(cutNarrowPassage);
   }
}
