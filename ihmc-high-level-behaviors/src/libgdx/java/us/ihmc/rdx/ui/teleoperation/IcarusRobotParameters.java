package us.ihmc.rdx.ui.teleoperation;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class IcarusRobotParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey trajectoryTime = keys.addDoubleKey("Trajectory time");

   public IcarusRobotParameters(String robotName)
   {
      super(keys, IcarusRobotParameters.class, StringUtils.capitalize(robotName));
   }

   public double getTrajectoryTime()
   {
      return get(trajectoryTime);
   }
}
