package us.ihmc.rdx.ui.teleoperation;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class RDXTeleoperationParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey trajectoryTime = keys.addDoubleKey("Trajectory time");
   public static final DoubleStoredPropertyKey pelvisHeightChangeVelocity = keys.addDoubleKey("Pelvis height change velocity");
   public static final DoubleStoredPropertyKey chestOrientationVelocity = keys.addDoubleKey("Chest orientation velocity");
   public static final DoubleStoredPropertyKey pelvisMinimumHeight = keys.addDoubleKey("Pelvis minimum height");
   public static final DoubleStoredPropertyKey pelvisMaximumHeight = keys.addDoubleKey("Pelvis maximum height");
   public static final DoubleStoredPropertyKey chestMinimumPitch = keys.addDoubleKey("Chest minimum pitch");
   public static final DoubleStoredPropertyKey chestMaximumPitch = keys.addDoubleKey("Chest maximum pitch");
   public static final DoubleStoredPropertyKey chestMinimumYaw = keys.addDoubleKey("Chest minimum yaw");
   public static final DoubleStoredPropertyKey chestMaximumYaw = keys.addDoubleKey("Chest maximum yaw");
   public static final BooleanStoredPropertyKey isPSIAdjustable = keys.addBooleanKey("Is PSI adjustable");

   public RDXTeleoperationParameters(String robotName)
   {
      super(keys, RDXTeleoperationParameters.class, StringUtils.capitalize(robotName));
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

   public boolean getPSIAdjustable()
   {
      return get(isPSIAdjustable);
   }
}
