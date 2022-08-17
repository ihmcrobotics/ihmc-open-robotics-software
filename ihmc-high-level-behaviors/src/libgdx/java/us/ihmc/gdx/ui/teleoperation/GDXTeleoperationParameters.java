package us.ihmc.gdx.ui.teleoperation;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class GDXTeleoperationParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey trajectoryTime = keys.addDoubleKey("Trajectory time");
   public static final DoubleStoredPropertyKey pelvisHeightChangeVelocity = keys.addDoubleKey("Pelvis height change velocity");
   public static final DoubleStoredPropertyKey chestOrientationVelocity = keys.addDoubleKey("Chest orientation velocity");
   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");
   public static final DoubleStoredPropertyKey pelvisMinimumHeight = keys.addDoubleKey("Pelvis minimum height");
   public static final DoubleStoredPropertyKey pelvisMaximumHeight = keys.addDoubleKey("Pelvis maximum height");
   public static final DoubleStoredPropertyKey chestMinimumPitch = keys.addDoubleKey("Chest minimum pitch");
   public static final DoubleStoredPropertyKey chestMaximumPitch = keys.addDoubleKey("Chest maximum pitch");
   public static final DoubleStoredPropertyKey chestMinimumYaw = keys.addDoubleKey("Chest minimum yaw");
   public static final DoubleStoredPropertyKey chestMaximumYaw = keys.addDoubleKey("Chest maximum yaw");
   public static final DoubleStoredPropertyKey turnAggressiveness = keys.addDoubleKey("Turn aggressiveness");
   public static final DoubleStoredPropertyKey straightStepLength = keys.addDoubleKey("Straight step length");
   public static final DoubleStoredPropertyKey straightStepWidth = keys.addDoubleKey("Straight step width");
   public static final DoubleStoredPropertyKey reverseStepLength = keys.addDoubleKey("Reverse step length");
   public static final DoubleStoredPropertyKey reverseStepWidth = keys.addDoubleKey("Reverse step width");
   public static final DoubleStoredPropertyKey shuffleStepLength = keys.addDoubleKey("Shuffle step length");
   public static final DoubleStoredPropertyKey shuffleStepWidth = keys.addDoubleKey("Shuffle step width");
   public static final DoubleStoredPropertyKey turningStepWidth = keys.addDoubleKey("Turning step width");
   public static final DoubleStoredPropertyKey footstepLengthMultiplier = keys.addDoubleKey("Footstep length multiplier");
   public static final BooleanStoredPropertyKey isPSIAdjustable = keys.addBooleanKey("Is PSI adjustable");

   public GDXTeleoperationParameters(String robotRepoName, String robotSubsequentPathToResourceFolder, String robotName)
   {
      super(keys, GDXTeleoperationParameters.class, robotRepoName, robotSubsequentPathToResourceFolder, StringUtils.capitalize(robotName));
   }

   public double getTrajectoryTime()
   {
      return get(trajectoryTime);
   }

   public double getPelvisHeightChangeVelocity()
   {
      return get(pelvisHeightChangeVelocity);
   }

   public double getChestOrientationVelocity()
   {
      return get(chestOrientationVelocity);
   }

   public double getSwingTime()
   {
      return get(swingTime);
   }

   public double getTransferTime()
   {
      return get(transferTime);
   }

   public double getPelvisMinimumHeight()
   {
      return get(pelvisMinimumHeight);
   }

   public double getPelvisMaximumHeight()
   {
      return get(pelvisMaximumHeight);
   }

   public double getChestMinimumPitch()
   {
      return get(chestMinimumPitch);
   }

   public double getChestMaximumPitch()
   {
      return get(chestMaximumPitch);
   }

   public double getChestMinimumYaw()
   {
      return get(chestMinimumYaw);
   }

   public double getChestMaximumYaw()
   {
      return get(chestMaximumYaw);
   }

   public double getTurnAggressiveness()
   {
      return get(turnAggressiveness);
   }

   public double getStraightStepLength()
   {
      return get(straightStepLength);
   }

   public double getStraightStepWidth()
   {
      return get(straightStepWidth);
   }

   public double getReverseStepLength()
   {
      return get(reverseStepLength);
   }

   public double getReverseStepWidth()
   {
      return get(reverseStepWidth);
   }

   public double getShuffleStepLength()
   {
      return get(shuffleStepLength);
   }

   public double getShuffleStepWidth()
   {
      return get(shuffleStepWidth);
   }

   public double getTurningStepWidth()
   {
      return get(turningStepWidth);
   }

   public double getFootstepLengthMultiplier()
   {
      return get(footstepLengthMultiplier);
   }

   public boolean getPSIAdjustable()
   {
      return get(isPSIAdjustable);
   }
}
