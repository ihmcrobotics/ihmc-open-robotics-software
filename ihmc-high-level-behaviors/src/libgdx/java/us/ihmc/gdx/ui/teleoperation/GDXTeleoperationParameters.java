package us.ihmc.gdx.ui.teleoperation;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class GDXTeleoperationParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey trajectoryTime = keys.addDoubleKey("Trajectory time");
   public static final DoubleStoredPropertyKey chestOrientationVelocity = keys.addDoubleKey("Chest orientation velocity");
   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");
   public static final DoubleStoredPropertyKey chestMinimumPitch = keys.addDoubleKey("Chest minimum pitch");
   public static final DoubleStoredPropertyKey chestMaximumPitch = keys.addDoubleKey("Chest maximum pitch");
   public static final DoubleStoredPropertyKey chestMinimumYaw = keys.addDoubleKey("Chest minimum yaw");
   public static final DoubleStoredPropertyKey chestMaximumYaw = keys.addDoubleKey("Chest maximum yaw");

   public GDXTeleoperationParameters(String robotRepoName, String robotSubsequentPathToResourceFolder, String robotName)
   {
      super(keys, GDXTeleoperationParameters.class, robotRepoName, robotSubsequentPathToResourceFolder, StringUtils.capitalize(robotName));
   }

   public double getTrajectoryTime()
   {
      return get(trajectoryTime);
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
}
