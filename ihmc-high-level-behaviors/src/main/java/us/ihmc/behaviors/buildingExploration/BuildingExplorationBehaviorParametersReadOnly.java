package us.ihmc.behaviors.buildingExploration;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorParameters.*;

public interface BuildingExplorationBehaviorParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getDistanceFromDoorToTransition()
   {
      return get(distanceFromDoorToTransition);
   }

   default double getDistanceFromStairsToTransition()
   {
      return get(distanceFromStairsToTransition);
   }
}
