package us.ihmc.wanderer.parameters;

import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.back_lbx;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.back_mby;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.back_ubz;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.forcedSideDependentJointNames;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.jointNames;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.l_leg_kny;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.l_leg_lax;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.l_leg_lhy;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.l_leg_mhx;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.l_leg_uay;
import static us.ihmc.wanderer.parameters.WandererOrderedJointNames.l_leg_uhz;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.transform.RigidBodyTransform;
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

/**
 * Created by dstephen on 2/14/14.
 */
public class WandererJointMap implements DRCRobotJointMap
{
   public static final String chestName = "utorso";
   public static final String pelvisName = "pelvis";
   public static final String headName = null;

   private final SpineJointName[] spineJoints = { SpineJointName.SPINE_ROLL, SpineJointName.SPINE_PITCH, SpineJointName.SPINE_YAW };
   private final LegJointName[] legJoints = { LegJointName.HIP_ROLL, LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.KNEE_PITCH, LegJointName.ANKLE_ROLL, LegJointName.ANKLE_PITCH };
   private final NeckJointName[] neckJoints = {};
   private final ArmJointName[] armJoints = {};


   protected final LinkedHashMap<String, JointRole> jointRoles = new LinkedHashMap<String, JointRole>();
   private final LinkedHashMap<String, ImmutablePair<RobotSide, LimbName>> limbNames = new LinkedHashMap<String, ImmutablePair<RobotSide, LimbName>>();

   private final LinkedHashMap<String, ImmutablePair<RobotSide, LegJointName>> legJointNames = new LinkedHashMap<String, ImmutablePair<RobotSide, LegJointName>>();
   private final LinkedHashMap<String, ImmutablePair<RobotSide, ArmJointName>> armJointNames = new LinkedHashMap<String, ImmutablePair<RobotSide, ArmJointName>>();
   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<String, SpineJointName>();
   private final LinkedHashMap<String, NeckJointName> neckJointNames = new LinkedHashMap<String, NeckJointName>();

   private final SideDependentList<EnumMap<LegJointName, String>> legJointStrings = SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<ArmJointName, String>> armJointStrings = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private final EnumMap<SpineJointName, String> spineJointStrings = new EnumMap<SpineJointName, String>(SpineJointName.class);

   private final SideDependentList<String> nameOfJointsBeforeThighs = new SideDependentList<String>();
   private String[] jointNamesBeforeFeet = new String[2];

   public WandererJointMap()
   {
      super();
      for (RobotSide robotSide : RobotSide.values())
      {
         String[] forcedSideJointNames = forcedSideDependentJointNames.get(robotSide);
         legJointNames.put(forcedSideJointNames[l_leg_mhx], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_ROLL));
         legJointNames.put(forcedSideJointNames[l_leg_uhz], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(forcedSideJointNames[l_leg_lhy], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_kny], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.KNEE_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_lax], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_ROLL));
         legJointNames.put(forcedSideJointNames[l_leg_uay], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_PITCH));

         String prefix = robotSide.getSideNameFirstLetter().toLowerCase();
         limbNames.put(null, new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         limbNames.put(prefix + "_foot", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.LEG));
      }

      spineJointNames.put(jointNames[back_lbx], SpineJointName.SPINE_ROLL);
      spineJointNames.put(jointNames[back_mby], SpineJointName.SPINE_PITCH);
      spineJointNames.put(jointNames[back_ubz], SpineJointName.SPINE_YAW);

      for (String legJointString : legJointNames.keySet())
      {
         RobotSide robotSide = legJointNames.get(legJointString).getLeft();
         LegJointName legJointName = legJointNames.get(legJointString).getRight();
         legJointStrings.get(robotSide).put(legJointName, legJointString);
         jointRoles.put(legJointString, JointRole.LEG);
      }

      for (String spineJointString : spineJointNames.keySet())
      {
         spineJointStrings.put(spineJointNames.get(spineJointString), spineJointString);
         jointRoles.put(spineJointString, JointRole.SPINE);
      }

      for (RobotSide robtSide : RobotSide.values)
      {
         nameOfJointsBeforeThighs.put(robtSide, legJointStrings.get(robtSide).get(LegJointName.HIP_PITCH));
      }

      jointNamesBeforeFeet[0] = getJointBeforeFootName(RobotSide.LEFT);
      jointNamesBeforeFeet[1] = getJointBeforeFootName(RobotSide.RIGHT);
   }

   @Override
   public String getModelName()
   {
      return "wanderer";
   }

   @Override
   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide robotSide)
   {
      return WandererPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide);
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return jointNames;
   }

   @Override
   public SpineJointName[] getSpineJointNames()
   {
      return spineJoints;
   }

   @Override public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public JointRole getJointRole(String jointName)
   {
      return jointRoles.get(jointName);
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
   public ImmutablePair<RobotSide, LimbName> getLimbName(String limbName)
   {
      return limbNames.get(limbName);
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
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      HashSet<String> lastSimulatedJoints = new HashSet<String>();

      // don't simulate children of ll_ankle_pulley, lr_ankle_pulley, rr_ankle_pulley and rl_ankle_pulley

      for (RobotSide robotSide : RobotSide.values())
      {
         String prefix = robotSide.getSideNameFirstLetter().toLowerCase();
         lastSimulatedJoints.add(prefix + "l_ankle_pulley");
         lastSimulatedJoints.add(prefix + "r_ankle_pulley");
      }

      return lastSimulatedJoints;
   }

   @Override
   public String getJointBeforeFootName(RobotSide robotSide)
   {
      return legJointStrings.get(robotSide).get(LegJointName.ANKLE_ROLL);
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
   public SideDependentList<String> getNameOfJointBeforeThighs()
   {
      return nameOfJointsBeforeThighs;
   }

   @Override
   public SideDependentList<String> getNameOfJointBeforeHands()
   {
      return null;
   }

   @Override
   public String getJointBeforeHandName(RobotSide robotSide)
   {
      return null;
   }

   protected LinkedHashMap<String, ImmutablePair<RobotSide, LegJointName>> getLegJointNamesMap()
   {
      return legJointNames;
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
      return null;
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
      for(RobotSide robotSide : RobotSide.values)
      {
         String jointBeforeFootName = getJointBeforeFootName(robotSide);
         if(jointBeforeFootName != null && jointBeforeFootName.equals(joineNameBeforeEndEffector))
         {
            return robotSide;
         }
      }
      throw new IllegalArgumentException(joineNameBeforeEndEffector + " was not listed as an end effector in " + this.getClass().getSimpleName());
   }
}
