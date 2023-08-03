package us.ihmc.perception.rapidRegions;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface RapidRegionsExtractorParametersBasics extends RapidRegionsExtractorParametersReadOnly, StoredPropertySetBasics
{
   default void setNormalPackRange(int normalPackRange)
   {
      set(RapidRegionsExtractorParameters.normalPackRange, normalPackRange);
   }

   default void setCentroidPackRange(int centroidPackRange)
   {
      set(RapidRegionsExtractorParameters.centroidPackRange, centroidPackRange);
   }

   default void setMergeRange(int mergeRange)
   {
      set(RapidRegionsExtractorParameters.mergeRange, mergeRange);
   }

   default void setMergeOrthogonalThreshold(double mergeOrthogonalThreshold)
   {
      set(RapidRegionsExtractorParameters.mergeOrthogonalThreshold, mergeOrthogonalThreshold);
   }

   default void setMergeDistanceThreshold(double mergeDistanceThreshold)
   {
      set(RapidRegionsExtractorParameters.mergeDistanceThreshold, mergeDistanceThreshold);
   }

   default void setMergeAngularThreshold(double mergeAngularThreshold)
   {
      set(RapidRegionsExtractorParameters.mergeAngularThreshold, mergeAngularThreshold);
   }

   default void setConnectionThreshold(int connectionThreshold)
   {
      set(RapidRegionsExtractorParameters.connectionThreshold, connectionThreshold);
   }

   default void setPatchSize(int patchSize)
   {
      set(RapidRegionsExtractorParameters.patchSize, patchSize);
   }

   default void setDeadPixelFilterPatchSize(int deadPixelFilterPatchSize)
   {
      set(RapidRegionsExtractorParameters.deadPixelFilterPatchSize, deadPixelFilterPatchSize);
   }

   default void setFocalLengthXPixels(double focalLengthXPixels)
   {
      set(RapidRegionsExtractorParameters.focalLengthXPixels, focalLengthXPixels);
   }

   default void setFocalLengthYPixels(double focalLengthYPixels)
   {
      set(RapidRegionsExtractorParameters.focalLengthYPixels, focalLengthYPixels);
   }

   default void setPrincipalOffsetXPixels(double principalOffsetXPixels)
   {
      set(RapidRegionsExtractorParameters.principalOffsetXPixels, principalOffsetXPixels);
   }

   default void setPrincipalOffsetYPixels(double principalOffsetYPixels)
   {
      set(RapidRegionsExtractorParameters.principalOffsetYPixels, principalOffsetYPixels);
   }

   default void setUseFilteredImage(boolean useFilteredImage)
   {
      set(RapidRegionsExtractorParameters.useFilteredImage, useFilteredImage);
   }

   default void setUseSVDNormals(boolean useSVDNormals)
   {
      set(RapidRegionsExtractorParameters.useSVDNormals, useSVDNormals);
   }

   default void setSVDReductionFactor(int svdReductionFactor)
   {
      set(RapidRegionsExtractorParameters.svdReductionFactor, svdReductionFactor);
   }

   default void setInternalSearchDepthLimit(int internalSearchDepthLimit)
   {
      set(RapidRegionsExtractorParameters.internalSearchDepthLimit, internalSearchDepthLimit);
   }

   default void setBoundarySearchDepthLimit(int boundarySearchDepthLimit)
   {
      set(RapidRegionsExtractorParameters.boundarySearchDepthLimit, boundarySearchDepthLimit);
   }

   default void setRegionMinPatches(int regionMinPatches)
   {
      set(RapidRegionsExtractorParameters.regionMinPatches, regionMinPatches);
   }

   default void setBoundaryMinPatches(int boundaryMinPatches)
   {
      set(RapidRegionsExtractorParameters.boundaryMinPatches, boundaryMinPatches);
   }

   default void setRegionGrowthFactor(double regionGrowthFactor)
   {
      set(RapidRegionsExtractorParameters.regionGrowthFactor, regionGrowthFactor);
   }
}
