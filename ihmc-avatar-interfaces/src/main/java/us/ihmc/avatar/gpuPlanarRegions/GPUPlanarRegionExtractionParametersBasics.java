package us.ihmc.avatar.gpuPlanarRegions;

import us.ihmc.tools.property.StoredPropertySetBasics;

public interface GPUPlanarRegionExtractionParametersBasics extends GPUPlanarRegionExtractionParametersReadOnly, StoredPropertySetBasics
{
   default void setMergeDistanceThreshold(double mergeDistanceThreshold)
   {
      set(GPUPlanarRegionExtractionParameters.mergeDistanceThreshold, mergeDistanceThreshold);
   }

   default void setMergeAngularThreshold(double mergeAngularThreshold)
   {
      set(GPUPlanarRegionExtractionParameters.mergeAngularThreshold, mergeAngularThreshold);
   }

   default void setFilterDisparityThreshold(double filterDisparityThreshold)
   {
      set(GPUPlanarRegionExtractionParameters.filterDisparityThreshold, filterDisparityThreshold);
   }

   default void setPatchSize(int patchSize)
   {
      set(GPUPlanarRegionExtractionParameters.patchSize, patchSize);
   }

   default void setDeadPixelFilterPatchSize(int deadPixelFilterPatchSize)
   {
      set(GPUPlanarRegionExtractionParameters.deadPixelFilterPatchSize, deadPixelFilterPatchSize);
   }

   default void setFocalLengthXPixels(double focalLengthXPixels)
   {
      set(GPUPlanarRegionExtractionParameters.focalLengthXPixels, focalLengthXPixels);
   }

   default void setFocalLengthYPixels(double focalLengthYPixels)
   {
      set(GPUPlanarRegionExtractionParameters.focalLengthYPixels, focalLengthYPixels);
   }

   default void setPrincipalOffsetXPixels(double principalOffsetXPixels)
   {
      set(GPUPlanarRegionExtractionParameters.principalOffsetXPixels, principalOffsetXPixels);
   }

   default void setPrincipalOffsetYPixels(double principalOffsetYPixels)
   {
      set(GPUPlanarRegionExtractionParameters.principalOffsetYPixels, principalOffsetYPixels);
   }

   default void setEarlyGaussianBlur(boolean earlyGaussianBlur)
   {
      set(GPUPlanarRegionExtractionParameters.earlyGaussianBlur, earlyGaussianBlur);
   }

   default void setUseFilteredImage(boolean useFilteredImage)
   {
      set(GPUPlanarRegionExtractionParameters.useFilteredImage, useFilteredImage);
   }

   default void setUseSVDNormals(boolean useSVDNormals)
   {
      set(GPUPlanarRegionExtractionParameters.useSVDNormals, useSVDNormals);
   }

   default void setSVDReductionFactor(int svdReductionFactor)
   {
      set(GPUPlanarRegionExtractionParameters.svdReductionFactor, svdReductionFactor);
   }

   default void setGaussianSize(int gaussianSize)
   {
      set(GPUPlanarRegionExtractionParameters.gaussianSize, gaussianSize);
   }

   default void setGaussianSigma(double gaussianSigma)
   {
      set(GPUPlanarRegionExtractionParameters.gaussianSigma, gaussianSigma);
   }

   default void setSearchDepthLimit(int searchDepthLimit)
   {
      set(GPUPlanarRegionExtractionParameters.searchDepthLimit, searchDepthLimit);
   }

   default void setRegionMinPatches(int regionMinPatches)
   {
      set(GPUPlanarRegionExtractionParameters.regionMinPatches, regionMinPatches);
   }

   default void setBoundaryMinPatches(int boundaryMinPatches)
   {
      set(GPUPlanarRegionExtractionParameters.boundaryMinPatches, boundaryMinPatches);
   }

   default void setRegionGrowthFactor(double regionGrowthFactor)
   {
      set(GPUPlanarRegionExtractionParameters.regionGrowthFactor, regionGrowthFactor);
   }
}
