package us.ihmc.perception.parameters;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-perception/src/main/resources/us/ihmc/perception/parameters/PointCloudSegmentationParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class PointCloudSegmentationParameters extends StoredPropertySet implements PointCloudSegmentationParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-perception/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-perception/src/main/generated-java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   /**
    * The segmentation divisor divides the message up into parts. A message with 2048
    * scan lines with a divisor of 8 will result in eight 256 scan line messages being
    * sent out.
    */
   public static final IntegerStoredPropertyKey segmentationDivisor = keys.addIntegerKey("Segmentation divisor");
   /**
    * The whole scan send frequency is how fast the whole scan gets published. So the
    * message frequency will be this times the segmentation divisor. So for a whole
    * scan frequency of 4 and divisor of 8, messages will get sent at 32 Hz.
    */
   public static final DoubleStoredPropertyKey wholeScanSendFrequency = keys.addDoubleKey("Whole scan send frequency");

   /**
    * Loads this property set.
    */
   public PointCloudSegmentationParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public PointCloudSegmentationParameters(String versionSpecifier)
   {
      this(PointCloudSegmentationParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public PointCloudSegmentationParameters(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String versionSuffix)
   {
      super(keys, classForLoading, PointCloudSegmentationParameters.class, directoryNameToAssumePresent, subsequentPathToResourceFolder, versionSuffix);
      load();
   }

   public PointCloudSegmentationParameters(StoredPropertySetReadOnly other)
   {
      super(keys, PointCloudSegmentationParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           PointCloudSegmentationParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
