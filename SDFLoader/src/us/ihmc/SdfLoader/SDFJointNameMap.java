package us.ihmc.SdfLoader;

import java.util.List;
import java.util.Set;

import javax.vecmath.Vector3d;

import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

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
   
   public String getFullSdfPelvisName();

   public String getChestName();

   public String getHeadName();

   public String getJointBeforeFootName(RobotSide robotSide);

   public String getJointBeforeHandName(RobotSide robotSide);

   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap();

   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide);

   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide);

   public boolean isTorqueVelocityLimitsEnabled();

   public Set<String> getLastSimulatedJoints();
}
