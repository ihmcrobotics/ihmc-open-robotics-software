package us.ihmc.behaviors.demo;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorParameters.*;

public interface BuildingExplorationBehaviorParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getDistanceFromDoorToTransition()
   {
      return get(distanceFromDoorToTransition);
   }
}
