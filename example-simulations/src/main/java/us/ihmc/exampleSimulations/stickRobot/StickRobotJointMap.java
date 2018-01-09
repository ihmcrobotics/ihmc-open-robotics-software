package us.ihmc.exampleSimulations.stickRobot;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;
import static us.ihmc.exampleSimulations.stickRobot.StickRobotOrderedJointMap.jointNames;
import org.apache.commons.lang3.tuple.ImmutablePair;

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

public class StickRobotJointMap implements DRCRobotJointMap
{

   private static final String chestName = "torso";
   private static final String pelvisName = "pelvis";
   private static final String fullPelvisNameInSdf = pelvisName;
   private static final String headName = "upperNeckPitchLink";
   private static final SideDependentList<String> handNames = new SideDependentList<>(getRobotSidePrefix(RobotSide.LEFT) + "Palm",
                                                                                      getRobotSidePrefix(RobotSide.RIGHT) + "Palm");

   private final LegJointName[] legJoints = {LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE_PITCH,
         LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL};
   private final ArmJointName[] armJoints;
   private final SpineJointName[] spineJoints = {SpineJointName.SPINE_YAW, SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL};
   private final NeckJointName[] neckJoints = {NeckJointName.PROXIMAL_NECK_PITCH, NeckJointName.DISTAL_NECK_YAW, NeckJointName.DISTAL_NECK_PITCH};

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

   public StickRobotJointMap()
   {

      armJoints = new ArmJointName[] {ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, ArmJointName.ELBOW_PITCH,
            ArmJointName.ELBOW_ROLL, ArmJointName.WRIST_ROLL, ArmJointName.FIRST_WRIST_PITCH};

      
      // populating the legJointNames,armJointNames, limbNames and neckJointNames hash maps 
      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = StickRobotOrderedJointMap.forcedSideDependentJointNames.get(robotSide);
         legJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftHipYaw],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftHipRoll],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_ROLL));
         legJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftHipPitch],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_PITCH));
         legJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftKneePitch],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.KNEE_PITCH));
         legJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftAnklePitch],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_PITCH));
         legJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftAnkleRoll],
                           new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_ROLL));

         armJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftShoulderPitch],
                           new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_PITCH));
         armJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftShoulderRoll],
                           new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_ROLL));
         armJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftShoulderYaw],
                           new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_YAW));
         armJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftElbowPitch],
                           new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_PITCH));
         armJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftForearmYaw],
                           new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_ROLL));
         armJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftWristRoll],
                           new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_ROLL));
         armJointNames.put(forcedSideJointNames[StickRobotOrderedJointMap.LeftWristPitch],
                           new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.FIRST_WRIST_PITCH));

         String prefix = getRobotSidePrefix(robotSide);

         limbNames.put(prefix + "Palm", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         limbNames.put(prefix + "Foot", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.LEG));
      }

      spineJointNames.put(jointNames[StickRobotOrderedJointMap.TorsoYaw], SpineJointName.SPINE_YAW);
      spineJointNames.put(jointNames[StickRobotOrderedJointMap.TorsoPitch], SpineJointName.SPINE_PITCH);
      spineJointNames.put(jointNames[StickRobotOrderedJointMap.TorsoRoll], SpineJointName.SPINE_ROLL);

      neckJointNames.put(jointNames[StickRobotOrderedJointMap.NeckPitch], NeckJointName.PROXIMAL_NECK_PITCH);
      neckJointNames.put(jointNames[StickRobotOrderedJointMap.NeckYaw], NeckJointName.DISTAL_NECK_YAW);
      neckJointNames.put(jointNames[StickRobotOrderedJointMap.NeckRoll], NeckJointName.DISTAL_NECK_PITCH);

      // assigning joint role to different joints like LEG, ARM, SPINE, NECK;
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

      //  no idea why and where this is used
      for (RobotSide robotSide : RobotSide.values)
      {
         nameOfJointsBeforeThighs.put(robotSide, legJointStrings.get(robotSide).get(LegJointName.HIP_PITCH));
         nameOfJointsBeforeHands.put(robotSide, armJointStrings.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH));
      }

      // joint before the left and right foot end effector
      jointNamesBeforeFeet[0] = getJointBeforeFootName(RobotSide.LEFT);
      jointNamesBeforeFeet[1] = getJointBeforeFootName(RobotSide.RIGHT);
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
   public String getModelName()
   {
      return "stickRobot";
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
   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide)
   {
      return StickRobotPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide);
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      return StickRobotPhysicalProperties.handControlFrameToWristTransforms.get(robotSide);
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
   public String getUnsanitizedRootJointInSdf()
   {
      return fullPelvisNameInSdf;
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
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      HashSet<String> lastSimulatedJoints = new HashSet<>();
      for (RobotSide robotSide : RobotSide.values)
         lastSimulatedJoints.add(armJointStrings.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH));
      return lastSimulatedJoints;
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

         String endOfArm = armJointStrings.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH);
         if(endOfArm != null && endOfArm.equals(joineNameBeforeEndEffector))
         {
            return robotSide;
         }
      }
      throw new IllegalArgumentException(joineNameBeforeEndEffector + " was not listed as an end effector in " + this.getClass().getSimpleName());
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
   public String getNameOfJointBeforeChest()
   {
      return spineJointStrings.get(SpineJointName.SPINE_ROLL);
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return StickRobotOrderedJointMap.jointNames;
   }

   @Override
   public String getLegJointName(RobotSide robotSide, LegJointName legJointName)
   {
      return null;
   }

   @Override
   public String getArmJointName(RobotSide robotSide, ArmJointName armJointName)
   {
      return null;
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
      return null;
   }

   @Override
   public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoVariableRegistry registry)
   {
      return null;
   }

}
