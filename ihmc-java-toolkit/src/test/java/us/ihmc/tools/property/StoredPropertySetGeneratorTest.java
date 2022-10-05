package us.ihmc.tools.property;

import us.ihmc.tools.property.*;

public class StoredPropertySetGeneratorTest extends StoredPropertySet implements StoredPropertySetGeneratorTestBasics
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String TO_RESOURCE_FOLDER = "ihmc-high-level-behaviors/src/libgdx/resources";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey theFirstBooleanProperty = keys.addBooleanKey("The first boolean property");
   public static final DoubleStoredPropertyKey theFirstDoubleProperty = keys.addDoubleKey("The first double property");
   public static final IntegerStoredPropertyKey theFirstIntegerProperty = keys.addIntegerKey("The first integer property");

   public StoredPropertySetGeneratorTest()
   {
      super(keys, StoredPropertySetGeneratorTest.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySetJavaGenerator generator = new StoredPropertySetJavaGenerator(StoredPropertySetGeneratorTest.class,
                                                                                    "ihmc-open-robotics-software",
                                                                                    "ihmc-java-toolkit/src/test/java");
      generator.generate();
   }
}
