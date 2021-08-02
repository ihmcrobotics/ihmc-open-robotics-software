package us.ihmc.behaviors.demo;

import us.ihmc.tools.property.*;

public class BuildingExplorationBehaviorParameters extends StoredPropertySet implements BuildingExplorationBehaviorParametersReadOnly
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String TO_RESOURCE_FOLDER = "ihmc-high-level-behaviors/src/main/resources";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey distanceFromDoorToTransition = keys.addDoubleKey("Distance from door to transition");
   public static final DoubleStoredPropertyKey distanceFromStairsToTransition = keys.addDoubleKey("Distance from stairs to transition");

   public BuildingExplorationBehaviorParameters()
   {
      super(keys, BuildingExplorationBehaviorParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, BuildingExplorationBehaviorParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      parameters.loadUnsafe();
      parameters.save();
   }
}
