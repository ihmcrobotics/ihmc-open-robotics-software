package us.ihmc.humanoidBehaviors.stairs;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class TraverseStairsBehaviorParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey pauseTime = keys.addDoubleKey("Pause time", 3.5);
   public static final IntegerStoredPropertyKey numberOfStairsPerExecution = keys.addIntegerKey("Number Of Stairs Per Execution", 2);
   public static final DoubleStoredPropertyKey planningTimeout = keys.addDoubleKey("Planning timeout", 10.0);

   // TODO look back at logs for these
   public static final DoubleStoredPropertyKey trajectoryTime = keys.addDoubleKey("Trajectory Time", 3.0);
   public static final DoubleStoredPropertyKey chestPitch = keys.addDoubleKey("Chest Pitch", Math.toRadians(10.0));
   public static final DoubleStoredPropertyKey headPitch = keys.addDoubleKey("Neck Pitch", Math.toRadians(48.0));
   public static final DoubleStoredPropertyKey amountToLowerFirstStepDown = keys.addDoubleKey("Amount to lower first step down", 0.01);
   public static final DoubleStoredPropertyKey heightToConsiderStepDown = keys.addDoubleKey("Height to consider step down", 0.07);

   public static final DoubleStoredPropertyKey xyProximityForCompletion = keys.addDoubleKey("XY Proximity For Completion", 0.8);

   public TraverseStairsBehaviorParameters()
   {
      super(keys, TraverseStairsBehaviorParameters.class, "ihmc-open-robotics-software", "ihmc-avatar-interfaces/src/behavior-clean-room/resources");
      load();
   }

   public static void main(String[] args)
   {
      TraverseStairsBehaviorParameters parameters = new TraverseStairsBehaviorParameters();
      parameters.save();
   }
}
