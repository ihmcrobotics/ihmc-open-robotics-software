package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class ExploreAreaBehaviorParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey turnChestTrajectoryDuration = keys.addDoubleKey("Turn chest trajectory duration");
   public static final DoubleStoredPropertyKey turnTrajectoryWaitTimeMulitplier = keys.addDoubleKey("Turn trajectory wait time multiplier");
   public static final DoubleStoredPropertyKey perceiveDuration = keys.addDoubleKey("Perceive duration");
   public static final DoubleStoredPropertyKey chestYawAmount = keys.addDoubleKey("Chest yaw amount");
   public static final DoubleStoredPropertyKey minimumDistanceBetweenObservationPoints = keys.addDoubleKey("Minimum distance between observation points");
   public static final DoubleStoredPropertyKey minDistanceToWalkIfPossible = keys.addDoubleKey("Minimum distance to walk if possible");

   public ExploreAreaBehaviorParameters()
   {
      super(keys,
            ExploreAreaBehaviorParameters.class,
            "ihmc-open-robotics-software",
            "ihmc-avatar-interfaces/src/behavior-clean-room/resources");

      load();
   }

   /**
    * Run to update file with new parameters.
    */
   public static void main(String[] args)
   {
      ExploreAreaBehaviorParameters planarRegionSLAMParameters = new ExploreAreaBehaviorParameters();
      planarRegionSLAMParameters.save();
   }
}
