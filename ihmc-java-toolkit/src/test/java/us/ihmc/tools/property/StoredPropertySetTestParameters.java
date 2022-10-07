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
   /**
    * The first boolean description.
    */
   public static final BooleanStoredPropertyKey booleanPropertyWithADescription = keys.addBooleanKey("Boolean property with a description");
   /**
    * The first integer description.
    */
   public static final DoubleStoredPropertyKey doublePropertyWithADescription = keys.addDoubleKey("Double property with a description");
   /**
    * The first integer description.
    */
   public static final IntegerStoredPropertyKey integerPropertyWithADescription = keys.addIntegerKey("Integer property with a description");
   /**
    * The double property with more stuff.
    */
   public static final DoubleStoredPropertyKey doublePropertyWithMoreStuff = keys.addDoubleKey("Double property with more stuff");
   /**
    * The integer property with more stuff.
    */
   public static final IntegerStoredPropertyKey integerPropertyWithMoreStuff = keys.addIntegerKey("Integer property with more stuff");
   /**
    * The integer property with discrete valid values.
    */
   public static final IntegerStoredPropertyKey integerPropertyWithDiscreteValidValues = keys.addIntegerKey("Integer property with discrete valid values");

   public StoredPropertySetTestParameters()
   {
      this("");
   }

   public StoredPropertySetTestParameters(String versionSpecifier)
   {
      super(keys, StoredPropertySetTestParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
      load();
   }

   public StoredPropertySetTestParameters(StoredPropertySetReadOnly other)
   {
      super(keys, StoredPropertySetTestParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
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
