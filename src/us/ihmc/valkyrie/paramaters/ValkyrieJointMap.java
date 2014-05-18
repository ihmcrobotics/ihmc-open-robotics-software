package us.ihmc.valkyrie.paramaters;

import static us.ihmc.valkyrie.paramaters.ValkyrieOrderedJointMap.jointNames;
import static us.ihmc.valkyrie.paramaters.ValkyrieOrderedJointMap.WaistExtensor;
import static us.ihmc.valkyrie.paramaters.ValkyrieOrderedJointMap.WaistLateralExtensor;
import static us.ihmc.valkyrie.paramaters.ValkyrieOrderedJointMap.WaistRotator;
import static us.ihmc.valkyrie.paramaters.ValkyrieOrderedJointMap.LowerNeckExtensor;
import static us.ihmc.valkyrie.paramaters.ValkyrieOrderedJointMap.UpperNeckExtensor;
import static us.ihmc.valkyrie.paramaters.ValkyrieOrderedJointMap.NeckRotator;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;

public class ValkyrieJointMap extends DRCRobotJointMap 
{
   
   public static final SideDependentList<String> jointBeforeThighNames = new SideDependentList<String>("LeftHipExtensor","RightHipExtensor");
   public static final String chestName = "v1Trunk";
   public static final String pelvisName = "v1Pelvis";
   public static final String spineRollJointName = jointNames[WaistLateralExtensor];
   public static final String spinePitchJointName = jointNames[WaistExtensor];
   public static final String spineYawJointName = jointNames[WaistRotator];
   public static final String lowerNeckPitchJointName = jointNames[LowerNeckExtensor];
   public static final String upperNeckPitchJointName = jointNames[UpperNeckExtensor];
   public static final String neckYawJointName = jointNames[NeckRotator];
   public static final String headName = "v1Head";
   
   private final LegJointName[] legJoints = { LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE, LegJointName.ANKLE_PITCH,
         LegJointName.ANKLE_ROLL };
   private final ArmJointName[] armJoints = { ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, ArmJointName.ELBOW_PITCH,
         ArmJointName.ELBOW_YAW, ArmJointName.WRIST_ROLL, ArmJointName.WRIST_PITCH };
   private final SpineJointName[] spineJoints = { SpineJointName.SPINE_YAW, SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL };
   private final NeckJointName[] neckJoints = { NeckJointName.LOWER_NECK_PITCH, NeckJointName.NECK_YAW, NeckJointName.UPPER_NECK_PITCH };

   private final LinkedHashMap<String, JointRole> jointRoles = new LinkedHashMap<String, JointRole>();
   private final LinkedHashMap<String, Pair<RobotSide, LegJointName>> legJointNames = new LinkedHashMap<String, Pair<RobotSide, LegJointName>>();
   private final LinkedHashMap<String, Pair<RobotSide, ArmJointName>> armJointNames = new LinkedHashMap<String, Pair<RobotSide, ArmJointName>>();
   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<String, SpineJointName>();
   private final LinkedHashMap<String, NeckJointName> neckJointNames = new LinkedHashMap<String, NeckJointName>();
   private final LinkedHashMap<String, Pair<RobotSide, LimbName>> limbNames = new LinkedHashMap<String, Pair<RobotSide, LimbName>>();
   private final SideDependentList<String> jointBeforeFeetNames = new SideDependentList<String>();
   
   private final ValkyrieContactPointParamaters contactPointParameters;

   public ValkyrieJointMap()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         legJointNames
               .put(robotSide.getCamelCaseNameForMiddleOfExpression() + "HipRotator", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "HipAdductor", new Pair<RobotSide, LegJointName>(robotSide,
               LegJointName.HIP_ROLL));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "HipExtensor", new Pair<RobotSide, LegJointName>(robotSide,
               LegJointName.HIP_PITCH));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "KneeExtensor", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.KNEE));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "AnkleExtensor", new Pair<RobotSide, LegJointName>(robotSide,
               LegJointName.ANKLE_PITCH));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "Ankle", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_ROLL));

         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ShoulderExtensor", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.SHOULDER_PITCH));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ShoulderAdductor", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.SHOULDER_ROLL));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ShoulderSupinator", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.SHOULDER_YAW));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ElbowExtensor", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.ELBOW_PITCH));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ForearmSupinator", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.ELBOW_YAW));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "WristExtensor", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.WRIST_ROLL));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "Wrist", new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_PITCH));

         String prefix = getRobotSidePrefix(robotSide);

         limbNames.put(prefix + "Palm", new Pair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         limbNames.put(prefix + "UpperFoot", new Pair<RobotSide, LimbName>(robotSide, LimbName.LEG));

         jointBeforeFeetNames.put(robotSide, robotSide.getCamelCaseNameForMiddleOfExpression() + "Ankle");
      }

      spineJointNames.put("WaistRotator", SpineJointName.SPINE_YAW);
      spineJointNames.put("WaistExtensor", SpineJointName.SPINE_PITCH);
      spineJointNames.put("WaistLateralExtensor", SpineJointName.SPINE_ROLL);

      neckJointNames.put("LowerNeckExtensor", NeckJointName.LOWER_NECK_PITCH);
      neckJointNames.put("NeckRotator", NeckJointName.NECK_YAW);
      neckJointNames.put("UpperNeckExtensor", NeckJointName.UPPER_NECK_PITCH);

      for (String legJoint : legJointNames.keySet())
      {
         jointRoles.put(legJoint, JointRole.LEG);
      }

      for (String armJoint : armJointNames.keySet())
      {
         jointRoles.put(armJoint, JointRole.ARM);
      }

      for (String spineJoint : spineJointNames.keySet())
      {
         jointRoles.put(spineJoint, JointRole.SPINE);
      }

      for (String neckJoint : neckJointNames.keySet())
      {
         jointRoles.put(neckJoint, JointRole.NECK);
      }
      contactPointParameters = new ValkyrieContactPointParamaters(this);
   }
   

   private String getRobotSidePrefix(RobotSide robotSide)
   {
      return (robotSide == RobotSide.LEFT) ? "v1Left" : "v1Right";
   }
   
   @Override
   public String getHighestNeckPitchJointName()
   {
      return lowerNeckPitchJointName;
   }
   
   @Override
   public String getNameOfJointBeforeHand(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForMiddleOfExpression() + "Wrist";
   }

   @Override
   public String getNameOfJointBeforeThigh(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForMiddleOfExpression() + "HipExtensor";
   }

   @Override
   public String getNameOfJointBeforeChest()
   {
      return "WaistLateralExtensor";
   }

   @Override
   public Pair<RobotSide, LegJointName> getLegJointName(String jointName)
   {
      return legJointNames.get(jointName);
   }

   @Override
   public Pair<RobotSide, ArmJointName> getArmJointName(String jointName)
   {
      return armJointNames.get(jointName);
   }

   @Override
   public Pair<RobotSide, LimbName> getLimbName(String limbName)
   {
      return limbNames.get(limbName);
   }

   @Override
   public JointRole getJointRole(String jointName)
   {
      return jointRoles.get(jointName);
   }

   @Override
   public NeckJointName getNeckJointName(String jointName)
   {
      return neckJointNames.get(jointName);
   }

   @Override
   public SpineJointName getSpineJointName(String jointName)
   {
      return spineJointNames.get(jointName);
   }

   @Override
   public String getPelvisName()
   {
      return pelvisName;
   }

   @Override
   public String getChestName()
   {
      return chestName;
   }

   @Override
   public String getHeadName()
   {
      return headName;
   }

   @Override
   public LegJointName[] getLegJointNames()
   {
      return legJoints;
   }

   @Override
   public ArmJointName[] getArmJointNames()
   {
      return armJoints;
   }

   @Override
   public SpineJointName[] getSpineJointNames()
   {
      return spineJoints;
   }

   @Override
   public NeckJointName[] getNeckJointNames()
   {
      return neckJoints;
   }

   @Override
   public String getJointBeforeFootName(RobotSide robotSide)
   {
      return jointBeforeFeetNames.get(robotSide);
   }

   @Override
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return contactPointParameters.getJointNameGroundContactPointMap();
   }

   @Override
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      HashSet<String> lastSimulatedJoints = new HashSet<>();
      lastSimulatedJoints.add("LeftWrist");
      lastSimulatedJoints.add("RightWrist");
      return lastSimulatedJoints;
   }

   @Override
   public String getModelName()
   {
      return "V1";
   }

   @Override
   public SideDependentList<String> getJointBeforeThighNames()
   {
      return jointBeforeThighNames;
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return ValkyrieOrderedJointMap.jointNames;
   }

   @Override
   public SideDependentList<Transform3D> getAnkleToSoleFrameTransform()
   {
      return ValkyriePhysicalProperties.ankleToSoleFrameTransforms;
   }
}
