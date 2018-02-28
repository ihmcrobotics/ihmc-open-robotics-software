package us.ihmc.robotics.partNames;

import us.ihmc.robotics.robotSide.RobotSide;

public interface HumanoidJointNameMap extends LeggedJointNameMap<RobotSide>
{
   @Override
   default RobotSide[] getRobotSegments()
   {
      return RobotSide.values;
   }
}
