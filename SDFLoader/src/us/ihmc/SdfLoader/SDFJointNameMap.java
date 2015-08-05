package us.ihmc.SdfLoader;

import java.util.List;
import java.util.Set;

import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.LimbName;
import us.ihmc.humanoidRobotics.partNames.NeckJointName;
import us.ihmc.humanoidRobotics.partNames.RobotSpecificJointNames;
import us.ihmc.humanoidRobotics.partNames.SpineJointName;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public interface SDFJointNameMap extends RobotSpecificJointNames
{
   public enum JointRole
   {
      LEG, ARM, SPINE, NECK;
   }

   public String getModelName();

   public JointRole getJointRole(String jointName);

   public ImmutablePair<RobotSide, LegJointName> getLegJointName(String jointName);

   public ImmutablePair<RobotSide, ArmJointName> getArmJointName(String jointName);

   public NeckJointName getNeckJointName(String jointName);

   public SpineJointName getSpineJointName(String jointName);

   public ImmutablePair<RobotSide, LimbName> getLimbName(String limbName);

   public String getPelvisName();
   
   public String getUnsanitizedRootJointInSdf();

   public String getChestName();

   public String getHeadName();

   public String getJointBeforeFootName(RobotSide robotSide);

   public String getJointBeforeHandName(RobotSide robotSide);

   public List<ImmutablePair<String, Vector3d>> getJointNameGroundContactPointMap();

   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide);

   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide);

   public boolean isTorqueVelocityLimitsEnabled();

   public Set<String> getLastSimulatedJoints();
}
