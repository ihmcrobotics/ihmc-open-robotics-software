package us.ihmc.behaviors.roomExploration.old;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.roomExploration.old.BuildingExplorationBehaviorParameters.*;

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
