package us.ihmc.humanoidBehaviors.stairs;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class TraverseStairsBehaviorParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey pauseTime = keys.addDoubleKey("Pause time", 20.0);
   public static final IntegerStoredPropertyKey numberOfStairsPerExecution = keys.addIntegerKey("Number Of Stairs Per Execution", 3);

   // TODO look back at logs for these
   public static final DoubleStoredPropertyKey chestPitchForAscension = keys.addDoubleKey("Chest Pitch For Ascension");
   public static final DoubleStoredPropertyKey chestPitchForDescension = keys.addDoubleKey("Chest Pitch For Decension");
   public static final DoubleStoredPropertyKey neckPitchForAscension = keys.addDoubleKey("Neck Pitch For Ascension");
   public static final DoubleStoredPropertyKey neckPitchForDescension = keys.addDoubleKey("Neck Pitch For Decension");

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
