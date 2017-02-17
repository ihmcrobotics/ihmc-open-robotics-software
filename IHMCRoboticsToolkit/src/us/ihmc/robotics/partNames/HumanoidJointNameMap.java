package us.ihmc.robotics.partNames;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public interface HumanoidJointNameMap extends JointNameMap
{
   public ImmutablePair<RobotSide, LegJointName> getLegJointName(String jointName);

   public ImmutablePair<RobotSide, ArmJointName> getArmJointName(String jointName);

   public ImmutablePair<RobotSide, LimbName> getLimbName(String limbName);

   public String getJointBeforeFootName(RobotSide robotSide);

   public String getJointBeforeHandName(RobotSide robotSide);

   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide);

   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide);
}
