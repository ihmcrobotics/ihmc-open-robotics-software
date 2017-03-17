package us.ihmc.atlas;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.forcedSideDependentJointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_elx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_ely;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wrx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wry;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wry2;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_akx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_aky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_kny;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.neck_ry;
import static us.ihmc.robotics.partNames.ArmJointName.ELBOW_PITCH;
import static us.ihmc.robotics.partNames.ArmJointName.ELBOW_ROLL;
import static us.ihmc.robotics.partNames.ArmJointName.FIRST_WRIST_PITCH;
import static us.ihmc.robotics.partNames.ArmJointName.SECOND_WRIST_PITCH;
import static us.ihmc.robotics.partNames.ArmJointName.SHOULDER_ROLL;
import static us.ihmc.robotics.partNames.ArmJointName.SHOULDER_YAW;
import static us.ihmc.robotics.partNames.ArmJointName.WRIST_ROLL;
import static us.ihmc.robotics.partNames.LegJointName.ANKLE_PITCH;
import static us.ihmc.robotics.partNames.LegJointName.ANKLE_ROLL;
import static us.ihmc.robotics.partNames.LegJointName.HIP_PITCH;
import static us.ihmc.robotics.partNames.LegJointName.HIP_ROLL;
import static us.ihmc.robotics.partNames.LegJointName.HIP_YAW;
import static us.ihmc.robotics.partNames.LegJointName.KNEE_PITCH;
import static us.ihmc.robotics.partNames.NeckJointName.PROXIMAL_NECK_PITCH;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_PITCH;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_ROLL;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_YAW;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.avatar.drcRobot.NewRobotPhysicalProperties;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.FootContactPoints;

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
   public static final SideDependentList<String> handNames = new SideDependentList<>();
   static
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handNames.put(robotSide, getRobotSidePrefix(robotSide) + "hand");
      }
   }

   private final LegJointName[] legJoints = {HIP_YAW, HIP_ROLL, HIP_PITCH, KNEE_PITCH, ANKLE_PITCH, ANKLE_ROLL};
   private final ArmJointName[] armJoints;
   private final SpineJointName[] spineJoints = {SPINE_PITCH, SPINE_ROLL, SPINE_YAW};
   private final NeckJointName[] neckJoints = {PROXIMAL_NECK_PITCH};

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
   private final NewRobotPhysicalProperties atlasPhysicalProperties;

   private final SideDependentList<String> nameOfJointsBeforeThighs = new SideDependentList<>();
   private final SideDependentList<String> nameOfJointsBeforeHands = new SideDependentList<>();

   private final String[] jointNamesBeforeFeet = new String[2];

   public AtlasJointMap(AtlasRobotVersion atlasVersion, NewRobotPhysicalProperties atlasPhysicalProperties)
   {
      this(atlasVersion, atlasPhysicalProperties, null, false);
   }

   public AtlasJointMap(AtlasRobotVersion atlasVersion, NewRobotPhysicalProperties atlasPhysicalProperties, FootContactPoints simulationContactPoints,
         boolean createAdditionalContactPoints)
   {
      this.atlasVersion = atlasVersion;
      this.atlasPhysicalProperties = atlasPhysicalProperties;

      if (atlasVersion != AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
         armJoints = new ArmJointName[] {SHOULDER_YAW, SHOULDER_ROLL, ELBOW_PITCH, ELBOW_ROLL, FIRST_WRIST_PITCH, WRIST_ROLL, SECOND_WRIST_PITCH};
      else
         armJoints = new ArmJointName[] {SHOULDER_YAW, SHOULDER_ROLL, ELBOW_PITCH, ELBOW_ROLL};

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = getRobotSidePrefix(robotSide);
         String[] forcedSideJointNames = forcedSideDependentJointNames.get(robotSide);

         legJointNames.put(forcedSideJointNames[l_leg_hpz], new ImmutablePair<RobotSide, LegJointName>(robotSide, HIP_YAW));
         legJointNames.put(forcedSideJointNames[l_leg_hpx], new ImmutablePair<RobotSide, LegJointName>(robotSide, HIP_ROLL));
         legJointNames.put(forcedSideJointNames[l_leg_hpy], new ImmutablePair<RobotSide, LegJointName>(robotSide, HIP_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_kny], new ImmutablePair<RobotSide, LegJointName>(robotSide, KNEE_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_aky], new ImmutablePair<RobotSide, LegJointName>(robotSide, ANKLE_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_akx], new ImmutablePair<RobotSide, LegJointName>(robotSide, ANKLE_ROLL));

         limbNames.put(prefix + "foot", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.LEG));

         armJointNames.put(forcedSideJointNames[l_arm_shz], new ImmutablePair<RobotSide, ArmJointName>(robotSide, SHOULDER_YAW));
         armJointNames.put(forcedSideJointNames[l_arm_shx], new ImmutablePair<RobotSide, ArmJointName>(robotSide, SHOULDER_ROLL));
         armJointNames.put(forcedSideJointNames[l_arm_ely], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ELBOW_PITCH));
         armJointNames.put(forcedSideJointNames[l_arm_elx], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ELBOW_ROLL));

         if (atlasVersion != AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
            limbNames.put(prefix + "hand", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         else
            limbNames.put(prefix + "larm", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.ARM));

         if (atlasVersion == AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
            continue;

         armJointNames.put(forcedSideJointNames[l_arm_wry], new ImmutablePair<RobotSide, ArmJointName>(robotSide, FIRST_WRIST_PITCH));
         armJointNames.put(forcedSideJointNames[l_arm_wrx], new ImmutablePair<RobotSide, ArmJointName>(robotSide, WRIST_ROLL));
         armJointNames.put(forcedSideJointNames[l_arm_wry2], new ImmutablePair<RobotSide, ArmJointName>(robotSide, SECOND_WRIST_PITCH));

      }

      spineJointNames.put(jointNames[back_bkz], SPINE_YAW);
      spineJointNames.put(jointNames[back_bky], SPINE_PITCH);
      spineJointNames.put(jointNames[back_bkx], SPINE_ROLL);
      neckJointNames.put(jointNames[neck_ry], PROXIMAL_NECK_PITCH);

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

      for (RobotSide robtSide : RobotSide.values)
      {
         nameOfJointsBeforeThighs.put(robtSide, legJointStrings.get(robtSide).get(HIP_PITCH));
         if (atlasVersion != AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
            nameOfJointsBeforeHands.put(robtSide, armJointStrings.get(robtSide).get(SECOND_WRIST_PITCH));
         else
            nameOfJointsBeforeHands.put(robtSide, armJointStrings.get(robtSide).get(ELBOW_ROLL));
      }

      jointNamesBeforeFeet[0] = getJointBeforeFootName(RobotSide.LEFT);
      jointNamesBeforeFeet[1] = getJointBeforeFootName(RobotSide.RIGHT);

      boolean createFootContactPoints = true;
      contactPointParameters = new AtlasContactPointParameters(this, atlasVersion, createFootContactPoints, simulationContactPoints, createAdditionalContactPoints);
   }

   @Override
   public SideDependentList<String> getNameOfJointBeforeHands()
   {
      if (atlasVersion != AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
         return nameOfJointsBeforeHands;
      return null;
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

   private static String getRobotSidePrefix(RobotSide robotSide)
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

   public String getHandName(RobotSide robotSide)
   {
      return handNames.get(robotSide);
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
   public List<ImmutablePair<String, Vector3D>> getJointNameGroundContactPointMap()
   {
      return contactPointParameters.getJointNameGroundContactPointMap();
   }

   @Override
   public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoVariableRegistry registry)
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

      if (!atlasVersion.getHandModel().isHandSimulated())
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
      return atlasPhysicalProperties.getSoleToAnkleFrameTransforms().get(robotSide);
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      RigidBodyTransform attachmentPlateToPalm = JMEDataTypeUtils.jmeTransformToTransform3D(atlasVersion.getOffsetFromAttachmentPlate(robotSide));
      RigidBodyTransform attachmentPlateToWrist = atlasPhysicalProperties.getHandAttachmentPlateToWristTransforms().get(robotSide);
      RigidBodyTransform handControlFrameToWristTranform = new RigidBodyTransform();
      handControlFrameToWristTranform.set(attachmentPlateToWrist);
      handControlFrameToWristTranform.multiply(attachmentPlateToPalm);

      Vector3D translation = new Vector3D();
      handControlFrameToWristTranform.getTranslation(translation);
      translation.scale(getModelScale());
      handControlFrameToWristTranform.setTranslation(translation);

      return handControlFrameToWristTranform;
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

   @Override
   public String[] getJointNamesBeforeFeet()
   {
      return jointNamesBeforeFeet;
   }

   @Override
   public Enum<?>[] getRobotSegments()
   {
      return RobotSide.values;
   }

   @Override
   public Enum<?> getEndEffectorsRobotSegment(String joineNameBeforeEndEffector)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String jointBeforeFootName = getJointBeforeFootName(robotSide);
         if (jointBeforeFootName != null && jointBeforeFootName.equals(joineNameBeforeEndEffector))
         {
            return robotSide;
         }

         String endOfArm;
         if (atlasVersion != AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
            endOfArm = armJointStrings.get(robotSide).get(SECOND_WRIST_PITCH);
         else
            endOfArm = armJointStrings.get(robotSide).get(ELBOW_ROLL);
         if (endOfArm != null && endOfArm.equals(joineNameBeforeEndEffector))
         {
            return robotSide;
         }
      }
      throw new IllegalArgumentException(joineNameBeforeEndEffector + " was not listed as an end effector in " + this.getClass().getSimpleName());
   }

   @Override
   public double getModelScale()
   {
      return atlasPhysicalProperties.getModelScale();
   }

   @Override
   public double getMassScalePower()
   {
      return atlasPhysicalProperties.getMassScalePower();
   }

   public NewRobotPhysicalProperties getPhysicalProperties()
   {
      return atlasPhysicalProperties;
   }

   public String[] getHighInertiaForStableSimulationJoints()
   {
      return new String[] {"hokuyo_joint"};
   }
}
