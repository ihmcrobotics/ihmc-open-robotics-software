package us.ihmc.behaviors.targetFollowing;

import us.ihmc.tools.property.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main to regenerate.
 */
public class TargetFollowingBehaviorParameters extends StoredPropertySet implements TargetFollowingBehaviorParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-high-level-behaviors/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-high-level-behaviors/src/main/java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey minimumDistanceToKeepFromTarget = keys.addDoubleKey("Minimum distance to keep from target");
   public static final DoubleStoredPropertyKey lookAndStepGoalUpdatePeriod = keys.addDoubleKey("Look and step goal update period");
   public static final DoubleStoredPropertyKey testLoopRadius = keys.addDoubleKey("Test loop radius");

   public TargetFollowingBehaviorParameters()
   {
      super(keys, TargetFollowingBehaviorParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           TargetFollowingBehaviorParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
