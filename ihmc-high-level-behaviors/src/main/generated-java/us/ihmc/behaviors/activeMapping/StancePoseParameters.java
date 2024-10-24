package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-high-level-behaviors/src/main/resources/us/ihmc/behaviors/activeMapping/StancePoseParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class StancePoseParameters extends StoredPropertySet implements StancePoseParametersBasics
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey contactCostWeight = keys.addDoubleKey("Contact cost weight");
   public static final DoubleStoredPropertyKey heightChangeCostWeight = keys.addDoubleKey("Height change cost weight");
   public static final DoubleStoredPropertyKey costImprovementForSwitch = keys.addDoubleKey("Cost improvement for switch");
   public static final DoubleStoredPropertyKey maxContactValue = keys.addDoubleKey("Max contact value");
   public static final DoubleStoredPropertyKey nominalStanceDistance = keys.addDoubleKey("Nominal stance distance");
   public static final IntegerStoredPropertyKey searchWindowSize = keys.addIntegerKey("Search window size");
   public static final DoubleStoredPropertyKey searchWindowResolution = keys.addDoubleKey("Search window resolution");

   /**
    * Loads this property set.
    */
   public StancePoseParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public StancePoseParameters(String versionSuffix)
   {
      this(StancePoseParameters.class, versionSuffix);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public StancePoseParameters(Class<?> classForLoading, String versionSuffix)
   {
      super(keys, classForLoading, StancePoseParameters.class, versionSuffix);
      load();
   }

   public StancePoseParameters(StoredPropertySetReadOnly other)
   {
      super(keys, StancePoseParameters.class, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, StancePoseParameters.class);
      parameters.generateJavaFiles();
   }
}
