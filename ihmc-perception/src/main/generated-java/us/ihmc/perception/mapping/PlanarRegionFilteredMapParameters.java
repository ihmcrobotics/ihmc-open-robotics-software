package us.ihmc.perception.mapping;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-avatar-interfaces/src/main/resources/us/ihmc/avatar/filteredMap/PlanarRegionFilteredMapParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class PlanarRegionFilteredMapParameters extends StoredPropertySet implements PlanarRegionFilteredMapParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-avatar-interfaces/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-avatar-interfaces/src/main/generated-java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey updateAlphaTowardsMatch = keys.addDoubleKey("Update Alpha towards match");
   public static final IntegerStoredPropertyKey angleThresholdBetweenNormals = keys.addIntegerKey("Angle threshold between normals");
   public static final DoubleStoredPropertyKey orthogonalDistanceThreshold = keys.addDoubleKey("Orthogonal distance threshold");
   public static final DoubleStoredPropertyKey maxInterRegionDistance = keys.addDoubleKey("Max Inter Region Distance");

   /**
    * Loads this property set.
    */
   public PlanarRegionFilteredMapParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public PlanarRegionFilteredMapParameters(String versionSpecifier)
   {
      this(PlanarRegionFilteredMapParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public PlanarRegionFilteredMapParameters(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String versionSuffix)
   {
      super(keys, classForLoading, PlanarRegionFilteredMapParameters.class, directoryNameToAssumePresent, subsequentPathToResourceFolder, versionSuffix);
      load();
   }

   public PlanarRegionFilteredMapParameters(StoredPropertySetReadOnly other)
   {
      super(keys, PlanarRegionFilteredMapParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           PlanarRegionFilteredMapParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
