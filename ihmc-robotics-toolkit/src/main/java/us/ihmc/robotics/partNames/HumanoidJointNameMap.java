package us.ihmc.robotics.partNames;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public interface HumanoidJointNameMap extends LeggedJointNameMap<RobotSide>
{
   ImmutablePair<RobotSide, ArmJointName> getArmJointName(String jointName);

   String getJointBeforeHandName(RobotSide robotSide);

   RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide);

   default RigidBodyTransform getSoleToParentFrameTransform(RobotSide robotSide)
   {
      return getSoleToAnkleFrameTransform(robotSide);
   }

   String getPelvisName();

   String getChestName();

   @Deprecated
   RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide);

   @Override
   default RobotSide[] getRobotSegments()
   {
      return RobotSide.values;
   }
}
