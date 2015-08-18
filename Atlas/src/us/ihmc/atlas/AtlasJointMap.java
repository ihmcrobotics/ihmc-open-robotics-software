package us.ihmc.atlas;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.*;
import static us.ihmc.humanoidRobotics.partNames.ArmJointName.*;
import static us.ihmc.humanoidRobotics.partNames.LegJointName.ANKLE_PITCH;
import static us.ihmc.humanoidRobotics.partNames.LegJointName.ANKLE_ROLL;
import static us.ihmc.humanoidRobotics.partNames.LegJointName.HIP_PITCH;
import static us.ihmc.humanoidRobotics.partNames.LegJointName.HIP_ROLL;
import static us.ihmc.humanoidRobotics.partNames.LegJointName.HIP_YAW;
import static us.ihmc.humanoidRobotics.partNames.LegJointName.KNEE;
import static us.ihmc.humanoidRobotics.partNames.NeckJointName.LOWER_NECK_PITCH;
import static us.ihmc.humanoidRobotics.partNames.SpineJointName.SPINE_PITCH;
import static us.ihmc.humanoidRobotics.partNames.SpineJointName.SPINE_ROLL;
import static us.ihmc.humanoidRobotics.partNames.SpineJointName.SPINE_YAW;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.JointRole;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.graphics3DAdapter.jme.util.JMEDataTypeUtils;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.LimbName;
import us.ihmc.humanoidRobotics.partNames.NeckJointName;
import us.ihmc.humanoidRobotics.partNames.SpineJointName;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class AtlasJointMap implements DRCRobotJointMap
{

   // Enable joint limits
   final public static boolean ENABLE_JOINT_VELOCITY_TORQUE_LIMITS = false;

   static
   {
      if (!ENABLE_JOINT_VELOCITY_TORQUE_LIMITS)
      {
         PrintTools.info(AtlasJointMap.class, "Running with torque and velocity limits disabled.");
      }
   }

   public static final String chestName = "utorso";
   public static final String pelvisName = "pelvis";
   public static final String headName = "head";

   private final LegJointName[] legJoints = { HIP_YAW, HIP_ROLL, HIP_PITCH, KNEE, ANKLE_PITCH, ANKLE_ROLL };
   private final ArmJointName[] armJoints = { SHOULDER_YAW, SHOULDER_ROLL, ELBOW_PITCH, ELBOW_ROLL, FIRST_WRIST_PITCH, WRIST_ROLL, SECOND_WRIST_PITCH };
   private final SpineJointName[] spineJoints = { SPINE_PITCH, SPINE_ROLL, SPINE_YAW };
   private final NeckJointName[] neckJoints = { LOWER_NECK_PITCH };

   private final LinkedHashMap<String, JointRole> jointRoles = new LinkedHashMap<String, JointRole>();
   private final LinkedHashMap<String, ImmutablePair<RobotSide, LimbName>> limbNames = new LinkedHashMap<String, ImmutablePair<RobotSide, LimbName>>();

   private final LinkedHashMap<String, ImmutablePair<RobotSide, LegJointName>> legJointNames = new LinkedHashMap<String, ImmutablePair<RobotSide, LegJointName>>();
   private final LinkedHashMap<String, ImmutablePair<RobotSide, ArmJointName>> armJointNames = new LinkedHashMap<String, ImmutablePair<RobotSide, ArmJointName>>();
   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<String, SpineJointName>();
   private final LinkedHashMap<String, NeckJointName> neckJointNames = new LinkedHashMap<String, NeckJointName>();

   private final SideDependentList<EnumMap<LegJointName, String>> legJointStrings = SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<ArmJointName, String>> armJointStrings = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private final EnumMap<SpineJointName, String> spineJointStrings = new EnumMap<>(SpineJointName.class);
   private final EnumMap<NeckJointName, String> neckJointStrings = new EnumMap<>(NeckJointName.class);

   private final AtlasContactPointParameters contactPointParameters;
   private final AtlasRobotVersion atlasVersion;
   
   private final SideDependentList<String> nameOfJointsBeforeThighs = new SideDependentList<>();
   private final SideDependentList<String> nameOfJointsBeforeHands = new SideDependentList<>();

   public AtlasJointMap(AtlasRobotVersion atlasVersion)
   {
      this.atlasVersion = atlasVersion;

      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = forcedSideDependentJointNames.get(robotSide);
         legJointNames.put(forcedSideJointNames[l_leg_hpz], new ImmutablePair<RobotSide, LegJointName>(robotSide, HIP_YAW));
         legJointNames.put(forcedSideJointNames[l_leg_hpx], new ImmutablePair<RobotSide, LegJointName>(robotSide, HIP_ROLL));
         legJointNames.put(forcedSideJointNames[l_leg_hpy], new ImmutablePair<RobotSide, LegJointName>(robotSide, HIP_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_kny], new ImmutablePair<RobotSide, LegJointName>(robotSide, KNEE));
         legJointNames.put(forcedSideJointNames[l_leg_aky], new ImmutablePair<RobotSide, LegJointName>(robotSide, ANKLE_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_akx], new ImmutablePair<RobotSide, LegJointName>(robotSide, ANKLE_ROLL));

         armJointNames.put(forcedSideJointNames[l_arm_shz], new ImmutablePair<RobotSide, ArmJointName>(robotSide, SHOULDER_YAW));
         armJointNames.put(forcedSideJointNames[l_arm_shx], new ImmutablePair<RobotSide, ArmJointName>(robotSide, SHOULDER_ROLL));
         armJointNames.put(forcedSideJointNames[l_arm_ely], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ELBOW_PITCH));
         armJointNames.put(forcedSideJointNames[l_arm_elx], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ELBOW_ROLL));
         armJointNames.put(forcedSideJointNames[l_arm_wry], new ImmutablePair<RobotSide, ArmJointName>(robotSide, FIRST_WRIST_PITCH));
         armJointNames.put(forcedSideJointNames[l_arm_wrx], new ImmutablePair<RobotSide, ArmJointName>(robotSide, WRIST_ROLL));
         armJointNames.put(forcedSideJointNames[l_arm_wry2], new ImmutablePair<RobotSide, ArmJointName>(robotSide, SECOND_WRIST_PITCH));

         String prefix = getRobotSidePrefix(robotSide);

         limbNames.put(prefix + "hand", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         limbNames.put(prefix + "foot", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.LEG));
      }

      spineJointNames.put(jointNames[back_bkz], SPINE_YAW);
      spineJointNames.put(jointNames[back_bky], SPINE_PITCH);
      spineJointNames.put(jointNames[back_bkx], SPINE_ROLL);
      neckJointNames.put(jointNames[neck_ry], LOWER_NECK_PITCH);

      for (String legJointString : legJointNames.keySet())
      {
         RobotSide robotSide = legJointNames.get(legJointString).getLeft();
         LegJointName legJointName = legJointNames.get(legJointString).getRight();
         legJointStrings.get(robotSide).put(legJointName, legJointString);
         jointRoles.put(legJointString, JointRole.LEG);
      }

      for (String armJointString : armJointNames.keySet())
      {
         RobotSide robotSide = armJointNames.get(armJointString).getLeft();
         ArmJointName armJointName = armJointNames.get(armJointString).getRight();
         armJointStrings.get(robotSide).put(armJointName, armJointString);
         jointRoles.put(armJointString, JointRole.ARM);
      }

      for (String spineJointString : spineJointNames.keySet())
      {
         spineJointStrings.put(spineJointNames.get(spineJointString), spineJointString);
         jointRoles.put(spineJointString, JointRole.SPINE);
      }

      for (String neckJointString : neckJointNames.keySet())
      {
         neckJointStrings.put(neckJointNames.get(neckJointString), neckJointString);
         jointRoles.put(neckJointString, JointRole.NECK);
      }

      boolean createFootContactPoints = true;
      contactPointParameters = new AtlasContactPointParameters(this, atlasVersion, createFootContactPoints);
      
      for (RobotSide robtSide : RobotSide.values)
      {
         nameOfJointsBeforeThighs.put(robtSide, legJointStrings.get(robtSide).get(HIP_PITCH));
         nameOfJointsBeforeHands.put(robtSide, armJointStrings.get(robtSide).get(SECOND_WRIST_PITCH));
      }
   }

   @Override
   public SideDependentList<String> getNameOfJointBeforeHands()
   {
      return nameOfJointsBeforeHands;
   }

   @Override
   public SideDependentList<String> getNameOfJointBeforeThighs()
   {
      return nameOfJointsBeforeThighs;
   }

   @Override
   public String getNameOfJointBeforeChest()
   {
      return spineJointStrings.get(SPINE_ROLL);
   }

   private String getRobotSidePrefix(RobotSide robotSide)
   {
      return (robotSide == RobotSide.LEFT) ? "l_" : "r_";
   }

   @Override
   public ImmutablePair<RobotSide, LegJointName> getLegJointName(String jointName)
   {
      return legJointNames.get(jointName);
   }

   @Override
   public ImmutablePair<RobotSide, ArmJointName> getArmJointName(String jointName)
   {
      return armJointNames.get(jointName);
   }

   @Override
   public ImmutablePair<RobotSide, LimbName> getLimbName(String limbName)
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
      return legJointStrings.get(robotSide).get(ANKLE_ROLL);
   }
   
   @Override
   public String getJointBeforeHandName(RobotSide robotSide)
   {
      return nameOfJointsBeforeHands.get(robotSide);
   }
   
   @Override
   public AtlasContactPointParameters getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public List<ImmutablePair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return contactPointParameters.getJointNameGroundContactPointMap();
   }

   @Override public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public String getModelName()
   {
      return atlasVersion.getModelName();
   }

   @Override
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return ENABLE_JOINT_VELOCITY_TORQUE_LIMITS;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      HashSet<String> lastSimulatedJoints = new HashSet<>();
      
      if(!atlasVersion.getHandModel().isHandSimulated())
      {
         for (RobotSide robotSide : RobotSide.values)
            lastSimulatedJoints.add(armJointStrings.get(robotSide).get(SECOND_WRIST_PITCH));
      }
      return lastSimulatedJoints;
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return jointNames;
   }

   @Override
   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide)
   {
      return AtlasPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide);
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      RigidBodyTransform attachmentPlateToPalm = JMEDataTypeUtils.jmeTransformToTransform3D(atlasVersion.getOffsetFromAttachmentPlate(robotSide));
      RigidBodyTransform attachmentPlateToWrist = AtlasPhysicalProperties.handAttachmentPlateToWristTransforms.get(robotSide);
      RigidBodyTransform handControlFrameToWristTranform = new RigidBodyTransform();
      handControlFrameToWristTranform.multiply(attachmentPlateToWrist, attachmentPlateToPalm);
      return handControlFrameToWristTranform;
//      return AtlasPhysicalProperties.handAttachmentPlateToWristTransforms.get(robotSide);
   }

   @Override
   public String getLegJointName(RobotSide robotSide, LegJointName legJointName)
   {
      return legJointStrings.get(robotSide).get(legJointName);
   }

   @Override
   public String getArmJointName(RobotSide robotSide, ArmJointName armJointName)
   {
      return armJointStrings.get(robotSide).get(armJointName);
   }

   @Override
   public String getNeckJointName(NeckJointName neckJointName)
   {
      return neckJointStrings.get(neckJointName);
   }

   @Override
   public String getSpineJointName(SpineJointName spineJointName)
   {
      return spineJointStrings.get(spineJointName);
   }

   @Override
   public String[] getPositionControlledJointsForSimulation()
   {
      return getOrderedJointNames();
   }

   @Override
   public String getUnsanitizedRootJointInSdf()
   {
      return pelvisName;
   }
}

