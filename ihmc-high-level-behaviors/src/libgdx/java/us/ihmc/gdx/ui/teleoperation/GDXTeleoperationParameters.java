package us.ihmc.gdx.ui.teleoperation;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class GDXTeleoperationParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey trajectoryTime = keys.addDoubleKey("Trajectory time");
   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");

   public GDXTeleoperationParameters(String robotRepoName, String robotSubsequentPathToResourceFolder, String robotName)
   {
      super(keys, GDXTeleoperationParameters.class, robotRepoName, robotSubsequentPathToResourceFolder, StringUtils.capitalize(robotName));
   }

   public double getTrajectoryTime()
   {
      return get(trajectoryTime);
   }

   public double getSwingTime()
   {
      return get(swingTime);
   }

   public double getTransferTime()
   {
      return get(transferTime);
   }
}
