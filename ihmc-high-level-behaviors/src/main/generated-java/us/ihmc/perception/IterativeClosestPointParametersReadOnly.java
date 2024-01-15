package us.ihmc.perception;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.IterativeClosestPointParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface IterativeClosestPointParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getIgnoreCorrespondencesOnEdges()
   {
      return get(ignoreCorrespondencesOnEdges);
   }

   default boolean getComputeObjectPoseWithTrack()
   {
      return get(computeObjectPoseWithTrack);
   }

   default boolean getSegmentPointCloudWithObjectShape()
   {
      return get(segmentPointCloudWithObjectShape);
   }

   default double getImageSegmentationRadius()
   {
      return get(imageSegmentationRadius);
   }

   default boolean getPrioritizeCorrespondencesByDistanceInsteadOfRandom()
   {
      return get(prioritizeCorrespondencesByDistanceInsteadOfRandom);
   }

   default double getIterationTransformDiscountFactor()
   {
      return get(iterationTransformDiscountFactor);
   }
}
