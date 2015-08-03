package us.ihmc.valkyrie.parameters;

import static us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap.jointNames;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.robotics.humanoidRobot.partNames.ArmJointName;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.humanoidRobot.partNames.LimbName;
import us.ihmc.robotics.humanoidRobot.partNames.NeckJointName;
import us.ihmc.robotics.humanoidRobot.partNames.SpineJointName;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class ValkyrieJointMap implements DRCRobotJointMap
{
   private final String chestName = "Torso";
   private final String pelvisName = "Pelvis";
   private final String fullPelvisNameInSdf = pelvisName;
   private final String headName = "UpperNeckPitchLink";

   private final LegJointName[] legJoints = { LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };
   private final ArmJointName[] armJoints;
   private final SpineJointName[] spineJoints = { SpineJointName.SPINE_YAW, SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL };
   private final NeckJointName[] neckJoints = { NeckJointName.LOWER_NECK_PITCH, NeckJointName.NECK_YAW, NeckJointName.UPPER_NECK_PITCH };

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

   private final ValkyrieContactPointParameters contactPointParameters;

   private final SideDependentList<String> nameOfJointsBeforeThighs = new SideDependentList<>();
   private final SideDependentList<String> nameOfJointsBeforeHands = new SideDependentList<>();

   public ValkyrieJointMap()
   {
      boolean selectedValkyrieVersionHasArms = ValkyrieConfigurationRoot.VALKYRIE_WITH_ARMS;
      if (selectedValkyrieVersionHasArms)
      {
         armJoints = new ArmJointName[]{ ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, 
               ArmJointName.ELBOW_PITCH, ArmJointName.ELBOW_ROLL, ArmJointName.WRIST_ROLL, ArmJointName.FIRST_WRIST_PITCH };
      }
      else
      {
         armJoints = new ArmJointName[] {};
      }
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = ValkyrieOrderedJointMap.forcedSideDependentJointNames.get(robotSide);
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftHipYaw], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftHipRoll], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_ROLL));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftHipPitch], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_PITCH));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftKneePitch], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.KNEE));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftAnklePitch], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_PITCH));
         legJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftAnkleRoll], new ImmutablePair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_ROLL));

         if (selectedValkyrieVersionHasArms)
         {
            armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftShoulderPitch], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_PITCH));
            armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftShoulderRoll], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_ROLL));
            armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftShoulderYaw], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_YAW));
            armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftElbowPitch], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_PITCH));
            armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftForearmYaw], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_ROLL));
            armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftWristRoll], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_ROLL));
            armJointNames.put(forcedSideJointNames[ValkyrieOrderedJointMap.LeftWristPitch], new ImmutablePair<RobotSide, ArmJointName>(robotSide, ArmJointName.FIRST_WRIST_PITCH));
         }
         String prefix = getRobotSidePrefix(robotSide);

         if (selectedValkyrieVersionHasArms)
         {
            limbNames.put(prefix + "Palm", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         }
         limbNames.put(prefix + "Foot", new ImmutablePair<RobotSide, LimbName>(robotSide, LimbName.LEG));
      }

      spineJointNames.put(jointNames[ValkyrieOrderedJointMap.TorsoYaw], SpineJointName.SPINE_YAW);
      spineJointNames.put(jointNames[ValkyrieOrderedJointMap.TorsoPitch], SpineJointName.SPINE_PITCH);
      spineJointNames.put(jointNames[ValkyrieOrderedJointMap.TorsoRoll], SpineJointName.SPINE_ROLL);

      neckJointNames.put(jointNames[ValkyrieOrderedJointMap.LowerNeckPitch], NeckJointName.LOWER_NECK_PITCH);
      neckJointNames.put(jointNames[ValkyrieOrderedJointMap.NeckYaw], NeckJointName.NECK_YAW);
      neckJointNames.put(jointNames[ValkyrieOrderedJointMap.UpperNeckPitch], NeckJointName.UPPER_NECK_PITCH);

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

      contactPointParameters = new ValkyrieContactPointParameters(this);
      
      for (RobotSide robtSide : RobotSide.values)
      {
         nameOfJointsBeforeThighs.put(robtSide, legJointStrings.get(robtSide).get(LegJointName.HIP_PITCH));
         nameOfJointsBeforeHands.put(robtSide, armJointStrings.get(robtSide).get(ArmJointName.FIRST_WRIST_PITCH));
      }
   }

   private String getRobotSidePrefix(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForMiddleOfExpression();// "Left" or "Right"
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
   public ValkyrieContactPointParameters getContactPointParameters()
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
      return ValkyriePhysicalProperties.soleToAnkleFrameTransforms.get(robotSide);
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      if (ValkyrieConfigurationRoot.VALKYRIE_WITH_ARMS)
         return ValkyriePhysicalProperties.handControlFrameToWristTransforms.get(robotSide);
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
      String[] ret = new String[]
      {
            getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_PITCH),
            getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_ROLL),
            getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_PITCH),
            getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_ROLL),
            getSpineJointName(SpineJointName.SPINE_PITCH),
            getSpineJointName(SpineJointName.SPINE_ROLL),
      };
      return ret;
   }

   @Override
   public String[] getPositionControlledJointsForSimulation()
   {
      String[] ret = new String[]
      {
            getNeckJointName(NeckJointName.UPPER_NECK_PITCH),
            getNeckJointName(NeckJointName.LOWER_NECK_PITCH),
            getNeckJointName(NeckJointName.NECK_YAW),
            
            "RightIndexFingerPitch1",
            "RightIndexFingerPitch2",
            "RightIndexFingerPitch3",
            "RightMiddleFingerPitch1",
            "RightMiddleFingerPitch2",
            "RightMiddleFingerPitch3",
            "RightPinkyPitch1",  
            "RightPinkyPitch2",  
            "RightPinkyPitch3",  
            "RightThumbRoll",    
            "RightThumbPitch1",  
            "RightThumbPitch2",  
            "RightThumbPitch3",
            "LeftIndexFingerPitch1", 
            "LeftIndexFingerPitch2", 
            "LeftIndexFingerPitch3", 
            "LeftMiddleFingerPitch1",
            "LeftMiddleFingerPitch2",
            "LeftMiddleFingerPitch3",
            "LeftPinkyPitch1",       
            "LeftPinkyPitch2",       
            "LeftPinkyPitch3",       
            "LeftThumbRoll",         
            "LeftThumbPitch1",       
            "LeftThumbPitch2",       
            "LeftThumbPitch3"    
      };
      
      return ret;
   }

   @Override
   public String getUnsanitizedRootJointInSdf()
   {
      return fullPelvisNameInSdf;
   }
}
