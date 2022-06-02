package us.ihmc.perception.rapidRegions;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.rapidRegions.GPUPlanarRegionExtractionParameters.*;

public interface GPUPlanarRegionExtractionParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getMergeDistanceThreshold()
   {
      return get(mergeDistanceThreshold);
   }

   default double getMergeAngularThreshold()
   {
      return get(mergeAngularThreshold);
   }

   default double getFilterDisparityThreshold()
   {
      return get(filterDisparityThreshold);
   }

   default int getPatchSize()
   {
      return get(patchSize);
   }

   default int getDeadPixelFilterPatchSize()
   {
      return get(deadPixelFilterPatchSize);
   }

   default double getFocalLengthXPixels()
   {
      return get(focalLengthXPixels);
   }

   default double getFocalLengthYPixels()
   {
      return get(focalLengthYPixels);
   }

   default double getPrincipalOffsetXPixels()
   {
      return get(principalOffsetXPixels);
   }

   default double getPrincipalOffsetYPixels()
   {
      return get(principalOffsetYPixels);
   }

   default boolean getEarlyGaussianBlur()
   {
      return get(earlyGaussianBlur);
   }

   default boolean getUseFilteredImage()
   {
      return get(useFilteredImage);
   }

   default boolean getUseSVDNormals()
   {
      return get(useSVDNormals);
   }

   default int getSVDReductionFactor()
   {
      return get(svdReductionFactor);
   }

   default int getGaussianSize()
   {
      return get(gaussianSize);
   }

   default double getGaussianSigma()
   {
      return get(gaussianSigma);
   }

   default int getSearchDepthLimit()
   {
      return get(searchDepthLimit);
   }

   default int getRegionMinPatches()
   {
      return get(regionMinPatches);
   }

   default int getBoundaryMinPatches()
   {
      return get(boundaryMinPatches);
   }

   default double getRegionGrowthFactor()
   {
      return get(regionGrowthFactor);
   }
}
