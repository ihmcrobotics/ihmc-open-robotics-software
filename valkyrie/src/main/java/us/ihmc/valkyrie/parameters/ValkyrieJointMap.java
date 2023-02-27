package us.ihmc.valkyrie.parameters;

import static us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap.jointNames;

import java.util.*;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieJointMap implements HumanoidJointNameMap
{
   private static final String chestName = "torso";
   private static final String pelvisName = "pelvis";
   private static final String fullPelvisNameInSdf = pelvisName;
   private static final String headName = "upperNeckPitchLink";
   private static final SideDependentList<String> footNames = new SideDependentList<>(getRobotSidePrefix(RobotSide.LEFT) + "Foot",
                                                                                      getRobotSidePrefix(RobotSide.RIGHT) + "Foot");

   private final ValkyrieRobotVersion robotVersion;
   private final LegJointName[] legJoints = {LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE_PITCH,
         LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL};
   private final ArmJointName[] armJoints;
   private final SpineJointName[] spineJoints = {SpineJointName.SPINE_YAW, SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL};
   private final NeckJointName[] neckJoints = {NeckJointName.PROXIMAL_NECK_PITCH, NeckJointName.DISTAL_NECK_YAW, NeckJointName.DISTAL_NECK_PITCH};
   private final SideDependentList<String> handNames = new SideDependentList<>();

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

   private final SideDependentList<String> nameOfJointsBeforeThighs = new SideDependentList<>();
   private final SideDependentList<String> nameOfJointsBeforeHands = new SideDependentList<>();
   private final String[] jointNamesBeforeFeet = new String[2];

   private final double modelScale;
   private final double massScalePower;
   private final ValkyriePhysicalProperties physicalProperties;

   public ValkyrieJointMap(ValkyriePhysicalProperties physicalProperties, ValkyrieRobotVersion robotVersion)
   {
      this.robotVersion = robotVersion;
      this.physicalProperties = physicalProperties;
      modelScale = physicalProperties.getModelSizeScale();
      massScalePower = physicalProperties.getModelMassScalePower();

      switch (robotVersion)
      {
         case DEFAULT:
         case FINGERLESS:
            armJoints = new ArmJointName[] {ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, ArmJointName.ELBOW_PITCH,
                  ArmJointName.ELBOW_ROLL, ArmJointName.WRIST_ROLL, ArmJointName.FIRST_WRIST_PITCH};
            break;
         case ARM_MASS_SIM:
            armJoints = new ArmJointName[] {ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, ArmJointName.ELBOW_PITCH};
            break;
         case ARMLESS:
         default:
            armJoints = new ArmJointName[] {};
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = ValkyrieOrderedJointMap.forcedSideDependentJointNames.get(robotSide);
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftHipYaw],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftHipRoll],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_ROLL));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftHipPitch],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_PITCH));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftKneePitch],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.KNEE_PITCH));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftAnklePitch],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_PITCH));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftAnkleRoll],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_ROLL));

         switch (robotVersion)
         {
            case ARMLESS:
               break;
            case DEFAULT:
            case FINGERLESS:
               armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftForearmYaw],
                                 new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_ROLL));
               armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftWristRoll],
                                 new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_ROLL));
               armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftWristPitch],
                                 new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.FIRST_WRIST_PITCH));
            case ARM_MASS_SIM:
               armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftShoulderPitch],
                                 new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_PITCH));
               armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftShoulderRoll],
                                 new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_ROLL));
               armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftShoulderYaw],
                                 new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_YAW));
               armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftElbowPitch],
                                 new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_PITCH));
         }

         String prefix = getRobotSidePrefix(robotSide);
         switch (robotVersion)
         {
            case DEFAULT:
            case FINGERLESS:
               String endEffectorName = prefix + "Palm";
               limbNames.put(endEffectorName, new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.ARM));
               handNames.put(robotSide, endEffectorName);
               break;
            case ARM_MASS_SIM:
               endEffectorName = prefix + "ElbowPitchLink";
               limbNames.put(endEffectorName, new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.ARM));
               handNames.put(robotSide, endEffectorName);
               break;
            default:
         }
         limbNames.put(prefix + "Foot", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.LEG));
      }

      spineJointNames.put(jointNames[ValkyrieOrderedJointMap.TorsoYaw], SpineJointName.SPINE_YAW);
      spineJointNames.put(jointNames[ValkyrieOrderedJointMap.TorsoPitch], SpineJointName.SPINE_PITCH);
      spineJointNames.put(jointNames[ValkyrieOrderedJointMap.TorsoRoll], SpineJointName.SPINE_ROLL);

      neckJointNames.put(jointNames[ValkyrieOrderedJointMap.LowerNeckPitch], NeckJointName.PROXIMAL_NECK_PITCH);
      neckJointNames.put(jointNames[ValkyrieOrderedJointMap.NeckYaw], NeckJointName.DISTAL_NECK_YAW);
      neckJointNames.put(jointNames[ValkyrieOrderedJointMap.UpperNeckPitch], NeckJointName.DISTAL_NECK_PITCH);

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
         nameOfJointsBeforeThighs.put(robtSide, legJointStrings.get(robtSide).get(LegJointName.HIP_PITCH));
         nameOfJointsBeforeHands.put(robtSide, armJointStrings.get(robtSide).get(ArmJointName.FIRST_WRIST_PITCH));
      }

      jointNamesBeforeFeet[0] = getJointBeforeFootName(RobotSide.LEFT);
      jointNamesBeforeFeet[1] = getJointBeforeFootName(RobotSide.RIGHT);
   }

   @Override
   public double getJointBLimit(String jointName)
   {
      if (jointName.contains("Wrist"))
         return 5.0;
      else
         return HumanoidJointNameMap.super.getJointBLimit(jointName);
   }

   @Override
   public double getModelScale()
   {
      return modelScale;
   }

   @Override
   public double getMassScalePower()
   {
      return massScalePower;
   }

   private static String getRobotSidePrefix(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForStartOfExpression();// "Left" or "Right"
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
      return spineJointStrings.get(SpineJointName.SPINE_ROLL);
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
   public String getHandName(RobotSide robotSide)
   {
      return handNames.get(robotSide);
   }

   @Override
   public String getFootName(RobotSide robotSide)
   {
      return footNames.get(robotSide);
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
      return legJointStrings.get(robotSide).get(LegJointName.ANKLE_ROLL);
   }

   @Override
   public String getJointBeforeHandName(RobotSide robotSide)
   {
      return nameOfJointsBeforeHands.get(robotSide);
   }

   @Override
   public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoRegistry registry)
   {
      return null;
   }

   @Override
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      switch (robotVersion)
      {
         case DEFAULT:
            HashSet<String> lastSimulatedJoints = new HashSet<>();
            for (RobotSide robotSide : RobotSide.values)
               lastSimulatedJoints.add(armJointStrings.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH));
            return lastSimulatedJoints;
         default:
            return Collections.emptySet();
      }
   }

   @Override
   public String getModelName()
   {
      return "valkyrie";
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return ValkyrieOrderedJointMap.jointNames;
   }

   @Override
   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide)
   {
      return physicalProperties.getSoleToAnkleFrameTransform(robotSide);
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      switch (robotVersion)
      {
         case DEFAULT:
         case FINGERLESS:
            return physicalProperties.getHandControlFrameToWristTransform(robotSide);
         case ARM_MASS_SIM:
            return physicalProperties.getHandControlFrameToArmMassSimTransform(robotSide);
         default:
            return null;
      }
   }

   @Override
   public String getLegJointName(RobotSide robotSide, LegJointName legJointName)
   {
      return legJointStrings.get(robotSide).get(legJointName);
   }

   public String getLegJointSuccessorName(RobotSide robotSide, LegJointName legJointName)
   {
      if (legJointName == LegJointName.ANKLE_ROLL)
         return getFootName(robotSide);
      else
         return getLegJointName(robotSide, legJointName) + "Link";
   }

   @Override
   public String getArmJointName(RobotSide robotSide, ArmJointName armJointName)
   {
      return armJointStrings.get(robotSide).get(armJointName);
   }

   public String getArmJointSuccessorName(RobotSide robotSide, ArmJointName armJointName)
   {
      String jointName = getArmJointName(robotSide, armJointName);

      if (jointName.equals(getNameOfJointBeforeHands().get(robotSide)))
         return getHandName(robotSide);
      else
         return jointName + "Link";
   }

   @Override
   public String getNeckJointName(NeckJointName neckJointName)
   {
      return neckJointStrings.get(neckJointName);
   }

   public String getNeckJointSuccessorName(NeckJointName neckJointName)
   {
      String jointName = getNeckJointName(neckJointName);

      if (neckJointName == NeckJointName.DISTAL_NECK_PITCH)
         return getHeadName();
      else
         return jointName + "Link";
   }

   @Override
   public String getSpineJointName(SpineJointName spineJointName)
   {
      return spineJointStrings.get(spineJointName);
   }

   public String getSpineJointSuccessorName(SpineJointName spineJointName)
   {
      String jointName = getSpineJointName(spineJointName);

      if (jointName.equals(getNameOfJointBeforeChest()))
         return getChestName();
      else
         return jointName + "Link";
   }

   public ValkyrieRobotVersion getRobotVersion()
   {
      return robotVersion;
   }

   public String[] getNamesOfJointsUsingOutputEncoder()
   {
      String[] ret = new String[] {
            //            getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_PITCH),
            //            getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_ROLL),
            //            getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_PITCH),
            //            getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_ROLL),
            getSpineJointName(SpineJointName.SPINE_PITCH), getSpineJointName(SpineJointName.SPINE_ROLL),
            getArmJointName(RobotSide.LEFT, ArmJointName.WRIST_ROLL), getArmJointName(RobotSide.LEFT, ArmJointName.FIRST_WRIST_PITCH),
            getArmJointName(RobotSide.RIGHT, ArmJointName.WRIST_ROLL), getArmJointName(RobotSide.RIGHT, ArmJointName.FIRST_WRIST_PITCH),};
      return ret;
   }

   @Override
   public String[] getPositionControlledJointsForSimulation()
   {
      List<String> allJoints = new ArrayList<>();
      allJoints.addAll(getArmJointNamesAsStrings());
      allJoints.addAll(getLegJointNamesAsStrings());
      allJoints.addAll(getSpineJointNamesAsStrings());
      allJoints.addAll(getNeckJointNamesAsStrings());
      return allJoints.toArray(new String[allJoints.size()]);
   }

   @Override
   public String getUnsanitizedRootJointInSdf()
   {
      return fullPelvisNameInSdf;
   }

   @Override
   public String[] getJointNamesBeforeFeet()
   {
      return jointNamesBeforeFeet;
   }

   @Override
   public RobotSide[] getRobotSegments()
   {
      return RobotSide.values;
   }

   @Override
   public RobotSide getEndEffectorsRobotSegment(String joineNameBeforeEndEffector)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String jointBeforeFootName = getJointBeforeFootName(robotSide);
         if (jointBeforeFootName != null && jointBeforeFootName.equals(joineNameBeforeEndEffector))
         {
            return robotSide;
         }

         String endOfArm = armJointStrings.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH);
         if (endOfArm != null && endOfArm.equals(joineNameBeforeEndEffector))
         {
            return robotSide;
         }
      }
      throw new IllegalArgumentException(joineNameBeforeEndEffector + " was not listed as an end effector in " + this.getClass().getSimpleName());
   }

   @Override
   public String[] getHighInertiaForStableSimulationJoints()
   {
      return new String[] {"hokuyo_joint"};
   }
}
