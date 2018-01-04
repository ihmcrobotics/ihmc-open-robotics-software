package us.ihmc.escher.parameters;

import static us.ihmc.escher.parameters.EscherOrderedJointMap.jointNames;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.escher.configuration.EscherConfigurationRoot;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class EscherJointMap implements DRCRobotJointMap
{
   private static final String chestName = "chest";
   private static final String pelvisName = "pelvis";
   private static final String fullPelvisNameInSdf = pelvisName;
   private static final String headName = "head";
   private static final SideDependentList<String> handNames = new SideDependentList<String>(getRobotSidePrefix(RobotSide.LEFT) + "_palm", getRobotSidePrefix(RobotSide.RIGHT) + "_palm");

   private final LegJointName[] legJoints = { LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE_PITCH, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };
   private final ArmJointName[] armJoints;
   private final SpineJointName[] spineJoints = { SpineJointName.SPINE_YAW};
   private final NeckJointName[] neckJoints = { NeckJointName.DISTAL_NECK_YAW, NeckJointName.DISTAL_NECK_PITCH };

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

   public EscherJointMap()
   {
      boolean selectedEscherVersionHasArms = EscherConfigurationRoot.ESCHER_WITH_ARMS;
      if (selectedEscherVersionHasArms)
      {
         armJoints = new ArmJointName[]{ ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW,
               ArmJointName.ELBOW_PITCH, ArmJointName.ELBOW_YAW, ArmJointName.FIRST_WRIST_PITCH, ArmJointName.WRIST_ROLL };
      }
      else
      {
         armJoints = new ArmJointName[] {};
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = EscherOrderedJointMap.forcedSideDependentJointNames.get(robotSide);
         legJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftHipYaw], new ImmutablePair<>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftHipRoll], new ImmutablePair<>(robotSide, LegJointName.HIP_ROLL));
         legJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftHipPitch], new ImmutablePair<>(robotSide, LegJointName.HIP_PITCH));
         legJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftKneePitch], new ImmutablePair<>(robotSide, LegJointName.KNEE_PITCH));
         legJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftAnklePitch], new ImmutablePair<>(robotSide, LegJointName.ANKLE_PITCH));
         legJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftAnkleRoll], new ImmutablePair<>(robotSide, LegJointName.ANKLE_ROLL));

         if (selectedEscherVersionHasArms)
         {
            armJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftShoulderPitch], new ImmutablePair<>(robotSide, ArmJointName.SHOULDER_PITCH));
            armJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftShoulderRoll], new ImmutablePair<>(robotSide, ArmJointName.SHOULDER_ROLL));
            armJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftShoulderYaw], new ImmutablePair<>(robotSide, ArmJointName.SHOULDER_YAW));
            armJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftElbowPitch], new ImmutablePair<>(robotSide, ArmJointName.ELBOW_PITCH));
            armJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftForearmYaw], new ImmutablePair<>(robotSide, ArmJointName.ELBOW_YAW));
            armJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftWristPitch], new ImmutablePair<>(robotSide, ArmJointName.FIRST_WRIST_PITCH));
            armJointNames.put(forcedSideJointNames[EscherOrderedJointMap.LeftWristRoll], new ImmutablePair<>(robotSide, ArmJointName.WRIST_ROLL));
         }
         String prefix = getRobotSidePrefix(robotSide);

         if (selectedEscherVersionHasArms)
         {
            limbNames.put(prefix + "_palm", new ImmutablePair<>(robotSide, LimbName.ARM));
         }
         limbNames.put(prefix + "_foot", new ImmutablePair<>(robotSide, LimbName.LEG));
      }

      spineJointNames.put(jointNames[EscherOrderedJointMap.TorsoYaw], SpineJointName.SPINE_YAW);

      neckJointNames.put(jointNames[EscherOrderedJointMap.NeckYaw], NeckJointName.DISTAL_NECK_YAW);
      neckJointNames.put(jointNames[EscherOrderedJointMap.NeckPitch], NeckJointName.DISTAL_NECK_PITCH);

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
         nameOfJointsBeforeHands.put(robtSide, armJointStrings.get(robtSide).get(ArmJointName.WRIST_ROLL));
      }

      jointNamesBeforeFeet[0] = getJointBeforeFootName(RobotSide.LEFT);
      jointNamesBeforeFeet[1] = getJointBeforeFootName(RobotSide.RIGHT);


   }

   private static String getRobotSidePrefix(RobotSide robotSide)
   {
      return robotSide.getShortLowerCaseName();// "l" or "r"
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
      return legJointStrings.get(robotSide).get(LegJointName.ANKLE_ROLL);
   }

   @Override
   public String getJointBeforeHandName(RobotSide robotSide)
   {
      return nameOfJointsBeforeHands.get(robotSide);
   }

   @Override public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoVariableRegistry registry)
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
      HashSet<String> lastSimulatedJoints = new HashSet<>();
      for (RobotSide robotSide : RobotSide.values)
         lastSimulatedJoints.add(armJointStrings.get(robotSide).get(ArmJointName.WRIST_ROLL));
      return lastSimulatedJoints;
   }

   @Override
   public String getModelName()
   {
      return "escher";
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return EscherOrderedJointMap.jointNames;
   }

   @Override
   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide)
   {
      return EscherPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide);
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      if (EscherConfigurationRoot.ESCHER_WITH_ARMS)
         return EscherPhysicalProperties.handControlFrameToWristTransforms.get(robotSide);
      else
         return null;
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

   public String[] getNamesOfJointsUsingOutputEncoder()
   {
//      ArrayList<String> jointNameList = new ArrayList<>();
//      for (RobotSide robotSide : RobotSide.values)
//      {
//         for (ArmJointName armJoint : armJoints)
//            jointNameList.add(getArmJointName(robotSide, armJoint));
//         for (LegJointName legJoint : legJoints)
//            jointNameList.add(getLegJointName(robotSide, legJoint));
//      }
//      for (SpineJointName spineJoint : spineJoints)
//         jointNameList.add(getSpineJointName(spineJoint));

//      String[] ret = jointNameList.toArray(new String[0]);

      String[] ret = new String[]
      {
//            getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_PITCH),
//            getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_ROLL),
//            getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_PITCH),
//            getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_ROLL),
            getSpineJointName(SpineJointName.SPINE_YAW),
      };
      return ret;
   }

   @Override
   public String[] getPositionControlledJointsForSimulation()
   {
      String[] ret = new String[]
      {
            getSpineJointName(SpineJointName.SPINE_YAW),
            getNeckJointName(NeckJointName.DISTAL_NECK_YAW),
            getNeckJointName(NeckJointName.DISTAL_NECK_PITCH),
            getArmJointName(RobotSide.LEFT, ArmJointName.SHOULDER_PITCH),
            getArmJointName(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL),
            getArmJointName(RobotSide.LEFT, ArmJointName.SHOULDER_YAW),
            getArmJointName(RobotSide.LEFT, ArmJointName.ELBOW_PITCH),
            getArmJointName(RobotSide.LEFT, ArmJointName.ELBOW_YAW),
            getArmJointName(RobotSide.LEFT, ArmJointName.FIRST_WRIST_PITCH),
            getArmJointName(RobotSide.LEFT, ArmJointName.WRIST_ROLL),
            getArmJointName(RobotSide.RIGHT, ArmJointName.SHOULDER_PITCH),
            getArmJointName(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL),
            getArmJointName(RobotSide.RIGHT, ArmJointName.SHOULDER_YAW),
            getArmJointName(RobotSide.RIGHT, ArmJointName.ELBOW_PITCH),
            getArmJointName(RobotSide.RIGHT, ArmJointName.ELBOW_YAW),
            getArmJointName(RobotSide.RIGHT, ArmJointName.FIRST_WRIST_PITCH),
            getArmJointName(RobotSide.RIGHT, ArmJointName.WRIST_ROLL),
      };

      return ret;
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
   public Enum<?>[] getRobotSegments()
   {
      return RobotSide.values;
   }

   @Override
   public Enum<?> getEndEffectorsRobotSegment(String joineNameBeforeEndEffector)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         String jointBeforeFootName = getJointBeforeFootName(robotSide);
         if(jointBeforeFootName != null && jointBeforeFootName.equals(joineNameBeforeEndEffector))
         {
            return robotSide;
         }

         String endOfArm = armJointStrings.get(robotSide).get(ArmJointName.WRIST_ROLL);
         if(endOfArm != null && endOfArm.equals(joineNameBeforeEndEffector))
         {
            return robotSide;
         }
      }
      throw new IllegalArgumentException(joineNameBeforeEndEffector + " was not listed as an end effector in " + this.getClass().getSimpleName());
   }
}
