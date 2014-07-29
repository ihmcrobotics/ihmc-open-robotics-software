package us.ihmc.SdfLoader;

import java.util.List;
import java.util.Set;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;

public interface SDFJointNameMap extends RobotSpecificJointNames
{
   public enum JointRole
   {
      LEG, ARM, SPINE, NECK;
   }

   public String getModelName();

   public JointRole getJointRole(String jointName);

   public Pair<RobotSide, LegJointName> getLegJointName(String jointName);

   public Pair<RobotSide, ArmJointName> getArmJointName(String jointName);

   public NeckJointName getNeckJointName(String jointName);

   public SpineJointName getSpineJointName(String jointName);

   public Pair<RobotSide, LimbName> getLimbName(String limbName);

   public String getPelvisName();

   public String getChestName();

   public String getHeadName();

   public String getJointBeforeFootName(RobotSide robotSide);

   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap();

   public Transform3D getSoleToAnkleFrameTransform(RobotSide robotSide);

   public Transform3D getHandControlFrameToWristTransform(RobotSide robotSide);

   public boolean isTorqueVelocityLimitsEnabled();

   public Set<String> getLastSimulatedJoints();
}
