package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main to regenerate.
 */
public class HeightMapParameters extends StoredPropertySet implements HeightMapParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-sensor-processing/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-sensor-processing/src/main/java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey gridResolutionXY = keys.addDoubleKey("Grid resolution XY");
   public static final DoubleStoredPropertyKey gridSizeXY = keys.addDoubleKey("Grid size XY");
   public static final DoubleStoredPropertyKey maxZ = keys.addDoubleKey("Max Z");
   public static final DoubleStoredPropertyKey nominalStandardDeviation = keys.addDoubleKey("Nominal standard deviation");
   public static final IntegerStoredPropertyKey maxPointsPerCell = keys.addIntegerKey("Max points per cell");
   public static final DoubleStoredPropertyKey mahalanobisScale = keys.addDoubleKey("Mahalanobis scale");

   public HeightMapParameters()
   {
      super(keys, HeightMapParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           HeightMapParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
