package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;

public interface QuadrupedInitialPositionParameters
{
   public double getInitialHeight();

   public double getInitialPosition(QuadrupedJointName joint);
}
