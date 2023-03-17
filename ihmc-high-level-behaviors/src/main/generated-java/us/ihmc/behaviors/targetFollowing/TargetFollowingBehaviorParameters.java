package us.ihmc.behaviors.targetFollowing;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-high-level-behaviors/src/main/resources/us/ihmc/behaviors/targetFollowing/TargetFollowingBehaviorParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class TargetFollowingBehaviorParameters extends StoredPropertySet implements TargetFollowingBehaviorParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-high-level-behaviors/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-high-level-behaviors/src/main/generated-java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey minimumDistanceToKeepFromTarget = keys.addDoubleKey("Minimum distance to keep from target");
   public static final DoubleStoredPropertyKey lookAndStepGoalUpdatePeriod = keys.addDoubleKey("Look and step goal update period");
   public static final DoubleStoredPropertyKey testLoopRadius = keys.addDoubleKey("Test loop radius");

   public TargetFollowingBehaviorParameters()
   {
      this("");
   }

   public TargetFollowingBehaviorParameters(String versionSpecifier)
   {
      super(keys, TargetFollowingBehaviorParameters.class, versionSpecifier);
      load();
   }

   public TargetFollowingBehaviorParameters(StoredPropertySetReadOnly other)
   {
      super(keys, TargetFollowingBehaviorParameters.class, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           TargetFollowingBehaviorParameters.class);
      parameters.generateJavaFiles();
   }
}
