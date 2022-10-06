package us.ihmc.tools.property;

import us.ihmc.tools.property.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main to regenerate.
 */
public class StoredPropertySetTestParameters extends StoredPropertySet implements StoredPropertySetTestParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-java-toolkit/src/test/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-java-toolkit/src/test/java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey theFirstBooleanProperty = keys.addBooleanKey("The first boolean property");
   public static final DoubleStoredPropertyKey theFirstDoubleProperty = keys.addDoubleKey("The first double property");
   public static final IntegerStoredPropertyKey theFirstIntegerProperty = keys.addIntegerKey("The first integer property");

   public StoredPropertySetTestParameters()
   {
      super(keys, StoredPropertySetTestParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           StoredPropertySetTestParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
