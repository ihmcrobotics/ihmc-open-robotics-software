package us.ihmc.perception.rapidRegions;

import us.ihmc.tools.property.*;

public class GPUPlanarRegionExtractionParameters extends StoredPropertySet implements GPUPlanarRegionExtractionParametersBasics
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String TO_RESOURCE_FOLDER = "ihmc-high-level-behaviors/src/libgdx/resources";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey mergeDistanceThreshold = keys.addDoubleKey("Merge distance threshold");
   public static final DoubleStoredPropertyKey mergeAngularThreshold = keys.addDoubleKey("Merge angular threshold");
   public static final DoubleStoredPropertyKey filterDisparityThreshold = keys.addDoubleKey("Filter disparity threshold");
   public static final IntegerStoredPropertyKey patchSize = keys.addIntegerKey("Patch size");
   public static final IntegerStoredPropertyKey deadPixelFilterPatchSize = keys.addIntegerKey("Dead pixel filter patch size");
   public static final DoubleStoredPropertyKey focalLengthXPixels = keys.addDoubleKey("Focal length X pixels");
   public static final DoubleStoredPropertyKey focalLengthYPixels = keys.addDoubleKey("Focal length Y pixels");
   public static final DoubleStoredPropertyKey principalOffsetXPixels = keys.addDoubleKey("Principal offset X pixels");
   public static final DoubleStoredPropertyKey principalOffsetYPixels = keys.addDoubleKey("Principal offset Y pixels");
   public static final BooleanStoredPropertyKey earlyGaussianBlur = keys.addBooleanKey("Early gaussian blur");
   public static final BooleanStoredPropertyKey useFilteredImage = keys.addBooleanKey("Use filtered image");
   public static final BooleanStoredPropertyKey useSVDNormals = keys.addBooleanKey("Use SVD normals");
   public static final IntegerStoredPropertyKey svdReductionFactor = keys.addIntegerKey("SVD reduction factor");
   public static final IntegerStoredPropertyKey gaussianSize = keys.addIntegerKey("Gaussian size");
   public static final DoubleStoredPropertyKey gaussianSigma = keys.addDoubleKey("Gaussian sigma");
   public static final IntegerStoredPropertyKey searchDepthLimit = keys.addIntegerKey("Search depth limit");
   public static final IntegerStoredPropertyKey regionMinPatches = keys.addIntegerKey("Region min patches");
   public static final IntegerStoredPropertyKey boundaryMinPatches = keys.addIntegerKey("Boundary min patches");
   public static final DoubleStoredPropertyKey regionGrowthFactor = keys.addDoubleKey("Region growth factor");

   public GPUPlanarRegionExtractionParameters()
   {
      super(keys, GPUPlanarRegionExtractionParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, GPUPlanarRegionExtractionParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      parameters.loadUnsafe();
      parameters.save();
   }
}
