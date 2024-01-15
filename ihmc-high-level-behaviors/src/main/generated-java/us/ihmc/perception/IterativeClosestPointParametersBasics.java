package us.ihmc.perception;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface IterativeClosestPointParametersBasics extends IterativeClosestPointParametersReadOnly, StoredPropertySetBasics
{
   default void setIgnoreCorrespondencesOnEdges(boolean ignoreCorrespondencesOnEdges)
   {
      set(IterativeClosestPointParameters.ignoreCorrespondencesOnEdges, ignoreCorrespondencesOnEdges);
   }

   default void setComputeObjectPoseWithTrack(boolean computeObjectPoseWithTrack)
   {
      set(IterativeClosestPointParameters.computeObjectPoseWithTrack, computeObjectPoseWithTrack);
   }

   default void setSegmentPointCloudWithObjectShape(boolean segmentPointCloudWithObjectShape)
   {
      set(IterativeClosestPointParameters.segmentPointCloudWithObjectShape, segmentPointCloudWithObjectShape);
   }

   default void setImageSegmentationRadius(double imageSegmentationRadius)
   {
      set(IterativeClosestPointParameters.imageSegmentationRadius, imageSegmentationRadius);
   }

   default void setPrioritizeCorrespondencesByDistanceInsteadOfRandom(boolean prioritizeCorrespondencesByDistanceInsteadOfRandom)
   {
      set(IterativeClosestPointParameters.prioritizeCorrespondencesByDistanceInsteadOfRandom, prioritizeCorrespondencesByDistanceInsteadOfRandom);
   }

   default void setIterationTransformDiscountFactor(double iterationTransformDiscountFactor)
   {
      set(IterativeClosestPointParameters.iterationTransformDiscountFactor, iterationTransformDiscountFactor);
   }
}
