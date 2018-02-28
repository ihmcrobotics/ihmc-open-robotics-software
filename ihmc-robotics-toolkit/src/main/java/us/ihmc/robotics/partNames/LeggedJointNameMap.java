package us.ihmc.robotics.partNames;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSegment;

public interface LeggedJointNameMap<E extends Enum<E> & RobotSegment<E>> extends JointNameMap
{
   ImmutablePair<E, LegJointName> getLegJointName(String jointName);

   ImmutablePair<E, ArmJointName> getArmJointName(String jointName);

   ImmutablePair<E, LimbName> getLimbName(String limbName);

   String getJointBeforeFootName(E robotSegment);

   String getJointBeforeHandName(E robotSegment);

   RigidBodyTransform getSoleToAnkleFrameTransform(E robotSegment);

   RigidBodyTransform getHandControlFrameToWristTransform(E robotSegment);
}
