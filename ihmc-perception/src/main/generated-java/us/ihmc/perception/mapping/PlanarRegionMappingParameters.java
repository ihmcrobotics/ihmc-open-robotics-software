package us.ihmc.perception.mapping;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-perception/src/main/resources/us/ihmc/perception/mapping/PlanarRegionMappingParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class PlanarRegionMappingParameters extends StoredPropertySet implements PlanarRegionMappingParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-perception/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-perception/src/main/generated-java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey updateAlphaTowardsMatch = keys.addDoubleKey("Update Alpha towards match");
   public static final DoubleStoredPropertyKey similarityThresholdBetweenNormals = keys.addDoubleKey("Similarity threshold between normals");
   public static final DoubleStoredPropertyKey orthogonalDistanceThreshold = keys.addDoubleKey("Orthogonal distance threshold");
   public static final DoubleStoredPropertyKey maxInterRegionDistance = keys.addDoubleKey("Max Inter Region Distance");
   public static final DoubleStoredPropertyKey minimumOverlapThreshold = keys.addDoubleKey("Minimum overlap threshold");
   public static final DoubleStoredPropertyKey minimumPlanarRegionArea = keys.addDoubleKey("Minimum planar region area");
   public static final DoubleStoredPropertyKey minimumBoundingBoxSize = keys.addDoubleKey("Minimum bounding box size");
   public static final DoubleStoredPropertyKey planeNoiseVariance = keys.addDoubleKey("Plane noise variance");
   public static final DoubleStoredPropertyKey odometryNoiseVariance = keys.addDoubleKey("Odometry noise variance");
   public static final DoubleStoredPropertyKey stateEstimatorNoiseVariance = keys.addDoubleKey("State estimator noise variance");
   public static final DoubleStoredPropertyKey bestMatchAngularThreshold = keys.addDoubleKey("Best match angular threshold");
   public static final DoubleStoredPropertyKey bestMatchDistanceThreshold = keys.addDoubleKey("Best match distance threshold");
   public static final DoubleStoredPropertyKey bestMinimumOverlapThreshold = keys.addDoubleKey("Best minimum overlap threshold");
   public static final DoubleStoredPropertyKey keyframeDistanceThreshold = keys.addDoubleKey("Keyframe distance threshold");
   public static final DoubleStoredPropertyKey keyframeAngularThreshold = keys.addDoubleKey("Keyframe angular threshold");
   public static final IntegerStoredPropertyKey icpMaxIterations = keys.addIntegerKey("ICP max iterations");
   public static final IntegerStoredPropertyKey icpMinMatches = keys.addIntegerKey("ICP min matches");
   public static final DoubleStoredPropertyKey icpTerminationRatio = keys.addDoubleKey("ICP termination ratio");
   public static final DoubleStoredPropertyKey icpErrorCutoff = keys.addDoubleKey("ICP error cutoff");
   public static final IntegerStoredPropertyKey minimumNumberOfTimesMatched = keys.addIntegerKey("Minimum number of times matched");

   /**
    * Loads this property set.
    */
   public PlanarRegionMappingParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public PlanarRegionMappingParameters(String versionSpecifier)
   {
      this(PlanarRegionMappingParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public PlanarRegionMappingParameters(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String versionSuffix)
   {
      super(keys, classForLoading, PlanarRegionMappingParameters.class, directoryNameToAssumePresent, subsequentPathToResourceFolder, versionSuffix);
      load();
   }

   public PlanarRegionMappingParameters(StoredPropertySetReadOnly other)
   {
      super(keys, PlanarRegionMappingParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           PlanarRegionMappingParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
