package us.ihmc.behaviors.buildingExploration.old;

import us.ihmc.tools.property.*;

public class BuildingExplorationBehaviorParameters extends StoredPropertySet implements BuildingExplorationBehaviorParametersReadOnly
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey distanceFromDoorToTransition = keys.addDoubleKey("Distance from door to transition");
   public static final DoubleStoredPropertyKey distanceFromStairsToTransition = keys.addDoubleKey("Distance from stairs to transition");

   public BuildingExplorationBehaviorParameters()
   {
      super(keys, BuildingExplorationBehaviorParameters.class);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, BuildingExplorationBehaviorParameters.class);
      parameters.loadUnsafe();
      parameters.save();
   }
}
