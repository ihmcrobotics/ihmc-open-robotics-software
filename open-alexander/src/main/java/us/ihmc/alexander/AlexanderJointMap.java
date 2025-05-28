package us.ihmc.alexander;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.alexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.*;

public class AlexanderJointMap implements HumanoidJointNameMap
{
   private final boolean hasHead;
   private final boolean hasArms;

   private final SideDependentList<String> handNames = new SideDependentList<>();
   private final SideDependentList<String> forearmNames = new SideDependentList<>();
   private final SideDependentList<String> footNames = new SideDependentList<>();
   private final String pelvisName = Links.PELVIS_LINK.getName();

   private final LinkedHashMap<String, ImmutablePair<RobotSide, ArmJointName>> armJointNames = new LinkedHashMap<>();
   private final LinkedHashMap<String, NeckJointName> neckJointNames = new LinkedHashMap<>();
   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<>();
   private final LinkedHashMap<String, ImmutablePair<RobotSide, LegJointName>> legJointNames = new LinkedHashMap<>();

   private final EnumMap<NeckJointName, String> neckJointStrings = new EnumMap<>(NeckJointName.class);
   private final SideDependentList<EnumMap<LegJointName, String>> legJointStrings = SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<ArmJointName, String>> armJointStrings = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private final EnumMap<SpineJointName, String> spineJointStrings = new EnumMap<>(SpineJointName.class);

   private final LegJointName[] legJoints = {LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE_PITCH,
                                             LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL};
   private final SpineJointName[] spineJoints = new SpineJointName[] {SpineJointName.SPINE_YAW};//, SpineJointName.SPINE_PITCH};
   private final NeckJointName[] neckJoints;
   private final SideDependentList<ArmJointName[]> armJoints = new SideDependentList<>();

   private final SideDependentList<RigidBodyTransform> handControlFrameToWristTransforms = new SideDependentList<>();
   private final AlexanderPhysicalProperties alexanderPhysicalProperties;
   private final SideDependentList<AlexanderArmConfiguration> armConfigurations;
   private final SideDependentList<String> nameOfJointsBeforeHands = new SideDependentList<>();
   private final LinkedHashMap<String, JointRole> jointRoles = new LinkedHashMap<>();
   private final String[] jointNamesBeforeFeet = new String[2];
   private final String[] jointNames;
   private final String fullPelvisNameInUrdf = pelvisName;

   private final HashSet<String> lastSimulatedJoints = new HashSet<>();

   public AlexanderJointMap(AlexanderPhysicalProperties alexanderPhysicalProperties,
                            SideDependentList<AlexanderArmConfiguration> armConfigurations,
                            boolean hasHead,
                            boolean hasArms)
   {
      this.alexanderPhysicalProperties = alexanderPhysicalProperties;
      this.armConfigurations = armConfigurations;
      this.hasHead = hasHead;
      this.hasArms = hasArms;

      List<String> jointNameList = new ArrayList<>();
      for (Joints joint : Joints.values())
      {
         if (joint.isPresent(armConfigurations))
            jointNameList.add(joint.name);
      }

      jointNames = jointNameList.toArray(new String[0]);

      if (hasArms)
         setArmJointNames();
      else
         armJoints.set(new SideDependentList<>(new ArmJointName[0], new ArmJointName[0]));

      setLegJointNames();

      if (hasHead)
      {
         neckJoints = new NeckJointName[] {NeckJointName.DISTAL_NECK_YAW, NeckJointName.DISTAL_NECK_PITCH};
         neckJointNames.put(Joints.NECK_Y.getName(), NeckJointName.DISTAL_NECK_PITCH);
         neckJointNames.put(Joints.NECK_Z.getName(), NeckJointName.DISTAL_NECK_YAW);
         neckJointStrings.put(NeckJointName.DISTAL_NECK_YAW, Joints.NECK_Z.getName());
         neckJointStrings.put(NeckJointName.DISTAL_NECK_PITCH, Joints.NECK_Y.getName());

         for (String neckJointString : neckJointNames.keySet())
         {
            jointRoles.put(neckJointString, JointRole.NECK);
         }
      }
      else
      {
         neckJoints = new NeckJointName[0];
      }

      spineJointNames.put(Joints.SPINE_Z.getName(), SpineJointName.SPINE_YAW);
//      spineJointNames.put(Joints.SPINE_Y.getName(), SpineJointName.SPINE_PITCH);
      spineJointStrings.put(SpineJointName.SPINE_YAW, Joints.SPINE_Z.getName());
//      spineJointStrings.put(SpineJointName.SPINE_PITCH, Joints.SPINE_Y.getName());

      for (String spineJointString : spineJointNames.keySet())
      {
         jointRoles.put(spineJointString, JointRole.SPINE);
      }

      if (hasArms)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            Joints lastSimulatedJoint = null;

            if (armConfigurations.get(robotSide) == AlexanderArmConfiguration.NUB)
            {
               Links hand = robotSide == RobotSide.LEFT ? Links.LEFT_ELBOW_PITCH_LINK : Links.RIGHT_ELBOW_PITCH_LINK;
               handNames.put(robotSide, hand.name);

               lastSimulatedJoint = robotSide == RobotSide.LEFT ? Joints.LEFT_ELBOW_Y : Joints.RIGHT_ELBOW_Y;

               handControlFrameToWristTransforms.put(robotSide, new RigidBodyTransform(new AxisAngle(), AlexanderNubHandModel.getElbowToControlFrame()));

               Joints jointBeforeHand = robotSide == RobotSide.LEFT ? Joints.LEFT_ELBOW_Y : Joints.RIGHT_ELBOW_Y;
               nameOfJointsBeforeHands.put(robotSide, jointBeforeHand.name);
            }
            else if (armConfigurations.get(robotSide) == AlexanderArmConfiguration.FOREARM)
            {
               Links hand = robotSide == RobotSide.LEFT ? Links.LEFT_GRIPPER_Z_LINK : Links.RIGHT_GRIPPER_Z_LINK;
               handNames.put(robotSide, hand.name);

               Links forearm = robotSide == RobotSide.LEFT ? Links.LEFT_WRIST_YAW_LINK : Links.RIGHT_WRIST_YAW_LINK;
               forearmNames.put(robotSide, forearm.name);

               lastSimulatedJoint = robotSide == RobotSide.LEFT ? Joints.LEFT_GRIPPER_Z : Joints.RIGHT_GRIPPER_Z;

               handControlFrameToWristTransforms.put(robotSide, alexanderPhysicalProperties.getHandControlFrameToWristTransform(robotSide));

               Joints jointBeforeHand = robotSide == RobotSide.LEFT ? Joints.LEFT_GRIPPER_Z : Joints.RIGHT_GRIPPER_Z;
               nameOfJointsBeforeHands.put(robotSide, jointBeforeHand.name);
            }

            if (lastSimulatedJoint != null)
            {
               lastSimulatedJoints.add(lastSimulatedJoint.name);
            }
         }
      }
   }

   public enum Joints
   {
      /* Left leg joints */
      LEFT_HIP_Z,
      LEFT_HIP_X,
      LEFT_HIP_Y,
      LEFT_KNEE_Y,
      LEFT_ANKLE_Y,
      LEFT_ANKLE_X,

      /* Right leg joints */
      RIGHT_HIP_Z,
      RIGHT_HIP_X,
      RIGHT_HIP_Y,
      RIGHT_KNEE_Y,
      RIGHT_ANKLE_Y,
      RIGHT_ANKLE_X,

      /* Spine joints */
      SPINE_Z,
//      SPINE_Y,

      NECK_Z,
      NECK_Y,
      /* Left shoulder joints */
      LEFT_SHOULDER_Y(RobotSide.LEFT, AlexanderArmConfiguration.NUB),
      LEFT_SHOULDER_X(RobotSide.LEFT, AlexanderArmConfiguration.NUB),
      LEFT_SHOULDER_Z(RobotSide.LEFT, AlexanderArmConfiguration.NUB),
      LEFT_ELBOW_Y(RobotSide.LEFT, AlexanderArmConfiguration.NUB),

      /* Left forearm + gripper joints */
      LEFT_WRIST_Z(RobotSide.LEFT, AlexanderArmConfiguration.FOREARM),
      LEFT_WRIST_X(RobotSide.LEFT, AlexanderArmConfiguration.FOREARM),
      LEFT_GRIPPER_Z(RobotSide.LEFT, AlexanderArmConfiguration.FOREARM),
      LEFT_GRIPPER_X1(RobotSide.LEFT, AlexanderArmConfiguration.FOREARM),
      LEFT_GRIPPER_X2(RobotSide.LEFT, AlexanderArmConfiguration.FOREARM),

      /* Left shoulder joints */
      RIGHT_SHOULDER_Y(RobotSide.RIGHT, AlexanderArmConfiguration.NUB),
      RIGHT_SHOULDER_X(RobotSide.RIGHT, AlexanderArmConfiguration.NUB),
      RIGHT_SHOULDER_Z(RobotSide.RIGHT, AlexanderArmConfiguration.NUB),
      RIGHT_ELBOW_Y(RobotSide.RIGHT, AlexanderArmConfiguration.NUB),

      /* Right forearm + gripper joints */
      RIGHT_WRIST_Z(RobotSide.RIGHT, AlexanderArmConfiguration.FOREARM),
      RIGHT_WRIST_X(RobotSide.RIGHT, AlexanderArmConfiguration.FOREARM),
      RIGHT_GRIPPER_Z(RobotSide.RIGHT, AlexanderArmConfiguration.FOREARM),
      RIGHT_GRIPPER_X1(RobotSide.RIGHT, AlexanderArmConfiguration.FOREARM),
      RIGHT_GRIPPER_X2(RobotSide.RIGHT, AlexanderArmConfiguration.FOREARM),
      ;

      private final String name = toString();
      private final RobotSide armJointSide;
      private final AlexanderArmConfiguration armConfiguration;

      Joints()
      {
         this(null, null);
      }

      Joints(RobotSide armJointSide, AlexanderArmConfiguration armConfiguration)
      {
         this.armJointSide = armJointSide;
         this.armConfiguration = armConfiguration;
      }

      public boolean isPresent(SideDependentList<AlexanderArmConfiguration> armConfigurations)
      {
         if (this.armConfiguration == null || armConfigurations == null)
            return true;
         else
            return armConfigurations.get(armJointSide).ordinal() >= this.armConfiguration.ordinal();
      }

      public String getName()
      {
         return name;
      }
   }

   public enum Links
   {
      HEAD_LINK,

      /* Left leg links */
      LEFT_HIP_YAW_LINK, LEFT_HIP_ROLL_LINK, LEFT_THIGH, LEFT_SHIN, LEFT_ANKLE_LINK, LEFT_FOOT,

      /* Right leg links */
      RIGHT_HIP_YAW_LINK, RIGHT_HIP_ROLL_LINK, RIGHT_THIGH, RIGHT_SHIN, RIGHT_ANKLE_LINK, RIGHT_FOOT,

      /* Spine links */
      PELVIS_LINK, SPINE_YAW_LINK, TORSO_LINK,

      /* Left shoulder links */
      LEFT_SHOULDER_PITCH_LINK, LEFT_SHOULDER_ROLL_LINK, LEFT_SHOULDER_YAW_LINK, LEFT_ELBOW_PITCH_LINK,

      /* Left forearm links */
      LEFT_WRIST_YAW_LINK, LEFT_WRIST_ROLL_LINK, LEFT_GRIPPER_Z_LINK, LEFT_GRIPPER_ROLL1_LINK, LEFT_GRIPPER_ROLL2_LINK,

      /* Right shoulder links */
      RIGHT_SHOULDER_PITCH_LINK, RIGHT_SHOULDER_ROLL_LINK, RIGHT_SHOULDER_YAW_LINK, RIGHT_ELBOW_PITCH_LINK,

      /* Right forearm links */
      RIGHT_WRIST_YAW_LINK, RIGHT_WRIST_ROLL_LINK, RIGHT_GRIPPER_Z_LINK, RIGHT_GRIPPER_ROLL1_LINK, RIGHT_GRIPPER_ROLL2_LINK,
      ;

      private final String name = toString();

      public String getName()
      {
         return name;
      }
   }

   private void setArmJointNames()
   {
      if (armConfigurations.get(RobotSide.LEFT) == AlexanderArmConfiguration.NUB || armConfigurations.get(RobotSide.LEFT) == AlexanderArmConfiguration.FOREARM)
      {
         armJointNames.put(Joints.LEFT_SHOULDER_Y.getName(), new ImmutablePair<>(RobotSide.LEFT, ArmJointName.SHOULDER_PITCH));
         armJointNames.put(Joints.LEFT_SHOULDER_X.getName(), new ImmutablePair<>(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL));
         armJointNames.put(Joints.LEFT_SHOULDER_Z.getName(), new ImmutablePair<>(RobotSide.LEFT, ArmJointName.SHOULDER_YAW));
         armJointNames.put(Joints.LEFT_ELBOW_Y.getName(), new ImmutablePair<>(RobotSide.LEFT, ArmJointName.ELBOW_PITCH));
      }
      if (armConfigurations.get(RobotSide.LEFT) == AlexanderArmConfiguration.FOREARM)
      {
         armJointNames.put(Joints.LEFT_WRIST_Z.getName(), new ImmutablePair<>(RobotSide.LEFT, ArmJointName.ELBOW_YAW));
         armJointNames.put(Joints.LEFT_WRIST_X.getName(), new ImmutablePair<>(RobotSide.LEFT, ArmJointName.WRIST_ROLL));
         armJointNames.put(Joints.LEFT_GRIPPER_Z.getName(), new ImmutablePair<>(RobotSide.LEFT, ArmJointName.WRIST_YAW));
      }

      if (armConfigurations.get(RobotSide.RIGHT) == AlexanderArmConfiguration.NUB
          || armConfigurations.get(RobotSide.RIGHT) == AlexanderArmConfiguration.FOREARM)
      {
         armJointNames.put(Joints.RIGHT_SHOULDER_Y.getName(), new ImmutablePair<>(RobotSide.RIGHT, ArmJointName.SHOULDER_PITCH));
         armJointNames.put(Joints.RIGHT_SHOULDER_X.getName(), new ImmutablePair<>(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL));
         armJointNames.put(Joints.RIGHT_SHOULDER_Z.getName(), new ImmutablePair<>(RobotSide.RIGHT, ArmJointName.SHOULDER_YAW));
         armJointNames.put(Joints.RIGHT_ELBOW_Y.getName(), new ImmutablePair<>(RobotSide.RIGHT, ArmJointName.ELBOW_PITCH));
      }
      if (armConfigurations.get(RobotSide.RIGHT) == AlexanderArmConfiguration.FOREARM)
      {
         armJointNames.put(Joints.RIGHT_WRIST_Z.getName(), new ImmutablePair<>(RobotSide.RIGHT, ArmJointName.ELBOW_YAW));
         armJointNames.put(Joints.RIGHT_WRIST_X.getName(), new ImmutablePair<>(RobotSide.RIGHT, ArmJointName.WRIST_ROLL));
         armJointNames.put(Joints.RIGHT_GRIPPER_Z.getName(), new ImmutablePair<>(RobotSide.RIGHT, ArmJointName.WRIST_YAW));
      }

      SideDependentList<List<ArmJointName>> armJointNameList = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());

      for (ImmutablePair<RobotSide, ArmJointName> armJointNamePair : armJointNames.values())
      {
         armJointNameList.get(armJointNamePair.getLeft()).add(armJointNamePair.getRight());
      }
      for (RobotSide robotSide : RobotSide.values())
      {
         armJoints.put(robotSide, armJointNameList.get(robotSide).toArray(new ArmJointName[0]));
      }

      for (String armJointString : armJointNames.keySet())
      {
         RobotSide robotSide = armJointNames.get(armJointString).getLeft();
         ArmJointName armJointName = armJointNames.get(armJointString).getRight();
         armJointStrings.get(robotSide).put(armJointName, armJointString);
         jointRoles.put(armJointString, JointRole.ARM);
      }
   }

   public void setLegJointNames()
   {
      // Leg mapping
      footNames.put(RobotSide.LEFT, Links.LEFT_FOOT.getName());
      footNames.put(RobotSide.RIGHT, Links.RIGHT_FOOT.getName());

      legJointNames.put(Joints.LEFT_HIP_Z.getName(), new ImmutablePair<>(RobotSide.LEFT, LegJointName.HIP_YAW));
      legJointNames.put(Joints.LEFT_HIP_X.getName(), new ImmutablePair<>(RobotSide.LEFT, LegJointName.HIP_ROLL));
      legJointNames.put(Joints.LEFT_HIP_Y.getName(), new ImmutablePair<>(RobotSide.LEFT, LegJointName.HIP_PITCH));
      legJointNames.put(Joints.LEFT_KNEE_Y.getName(), new ImmutablePair<>(RobotSide.LEFT, LegJointName.KNEE_PITCH));
      legJointNames.put(Joints.LEFT_ANKLE_Y.getName(), new ImmutablePair<>(RobotSide.LEFT, LegJointName.ANKLE_PITCH));
      legJointNames.put(Joints.LEFT_ANKLE_X.getName(), new ImmutablePair<>(RobotSide.LEFT, LegJointName.ANKLE_ROLL));

      legJointNames.put(Joints.RIGHT_HIP_Z.getName(), new ImmutablePair<>(RobotSide.RIGHT, LegJointName.HIP_YAW));
      legJointNames.put(Joints.RIGHT_HIP_X.getName(), new ImmutablePair<>(RobotSide.RIGHT, LegJointName.HIP_ROLL));
      legJointNames.put(Joints.RIGHT_HIP_Y.getName(), new ImmutablePair<>(RobotSide.RIGHT, LegJointName.HIP_PITCH));
      legJointNames.put(Joints.RIGHT_KNEE_Y.getName(), new ImmutablePair<>(RobotSide.RIGHT, LegJointName.KNEE_PITCH));
      legJointNames.put(Joints.RIGHT_ANKLE_Y.getName(), new ImmutablePair<>(RobotSide.RIGHT, LegJointName.ANKLE_PITCH));
      legJointNames.put(Joints.RIGHT_ANKLE_X.getName(), new ImmutablePair<>(RobotSide.RIGHT, LegJointName.ANKLE_ROLL));

      for (String legJointString : legJointNames.keySet())
      {
         RobotSide robotSide = legJointNames.get(legJointString).getLeft();
         LegJointName legJointName = legJointNames.get(legJointString).getRight();
         legJointStrings.get(robotSide).put(legJointName, legJointString);
         jointRoles.put(legJointString, JointRole.LEG);
      }

      jointNamesBeforeFeet[0] = getJointBeforeFootName(RobotSide.LEFT);
      jointNamesBeforeFeet[1] = getJointBeforeFootName(RobotSide.RIGHT);
   }

   public AlexanderArmConfiguration getArmConfiguration(RobotSide robotSide)
   {
      return armConfigurations.get(robotSide);
   }

   @Override
   public ImmutablePair<RobotSide, ArmJointName> getArmJointName(String jointName)
   {
      return armJointNames.get(jointName);
   }

   @Override
   public SideDependentList<String> getNameOfJointBeforeHands()
   {
      return nameOfJointsBeforeHands;
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      return handControlFrameToWristTransforms.get(robotSide);
   }

   @Override
   public String getPelvisName()
   {
      return pelvisName;
   }

   @Override
   public String getChestName()
   {
      return Links.TORSO_LINK.getName();
   }

   @Override
   public String getNameOfJointBeforeChest()
   {
      return Joints.SPINE_Z.getName();
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return jointNames;
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
      List<String> allJoints = new ArrayList<>();
      allJoints.addAll(getLegJointNamesAsStrings());
      allJoints.addAll(getSpineJointNamesAsStrings());
      return allJoints.toArray(new String[allJoints.size()]);
   }

   @Override
   public String getHandName(RobotSide robotSide)
   {
      return handNames.get(robotSide);
   }

   @Override
   public String getForearmName(RobotSide robotSide)
   {
      return forearmNames.get(robotSide);
   }

   @Override
   public String getFootName(RobotSide robotSide)
   {
      return footNames.get(robotSide);
   }

   @Override
   public ImmutablePair<RobotSide, LegJointName> getLegJointName(String jointName)
   {
      return legJointNames.get(jointName);
   }

   @Override
   public String getJointBeforeFootName(RobotSide robotSide)
   {
      return legJointStrings.get(robotSide).get(LegJointName.ANKLE_ROLL);
   }

   @Override
   public RigidBodyTransform getSoleToParentFrameTransform(RobotSide robotSide)
   {
      return alexanderPhysicalProperties.getSoleToAnkleFrameTransforms().get(robotSide);
   }

   @Override
   public String getModelName()
   {
      return "alexander";
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
   public String getUnsanitizedRootJointInSdf()
   {
      return fullPelvisNameInUrdf;
   }

   @Override
   public String getHeadName()
   {
      return Links.HEAD_LINK.getName();
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      return lastSimulatedJoints;
   }

   @Override
   public String[] getJointNamesBeforeFeet()
   {
      return jointNamesBeforeFeet;
   }

   @Override
   public RobotSide getEndEffectorsRobotSegment(String jointNameBeforeEndEffector)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String jointBeforeFootName = getJointBeforeFootName(robotSide);
         if (jointBeforeFootName != null && jointBeforeFootName.equals(jointNameBeforeEndEffector))
         {
            return robotSide;
         }

         if (jointNameBeforeEndEffector.contains(robotSide.getUpperCaseName()))
         {
            return robotSide;
         }
      }

      throw new IllegalArgumentException(jointNameBeforeEndEffector + " was not listed as an end effector in " + this.getClass().getSimpleName());
   }

   @Override
   public LegJointName[] getLegJointNames()
   {
      return legJoints;
   }

   @Override
   public ArmJointName[] getArmJointNames()
   {
      ArmJointName[] leftArmJoints = armJoints.get(RobotSide.LEFT);
      ArmJointName[] rightArmJoints = armJoints.get(RobotSide.RIGHT);
      // This seems to be fine when the configuration is asymmetric
      return leftArmJoints.length > rightArmJoints.length ? leftArmJoints : rightArmJoints;
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

   public boolean hasCycloidForearm(RobotSide side)
   {
      return switch (side)
      {
         case LEFT -> hasLeftCycloidForearm();
         case RIGHT -> hasRightCycloidForearm();
      };
   }

   public boolean hasLeftCycloidForearm()
   {
      if (armConfigurations == null)
         return false;
      return armConfigurations.get(RobotSide.LEFT) == AlexanderArmConfiguration.FOREARM;
   }

   public boolean hasRightCycloidForearm()
   {
      if (armConfigurations == null)
         return false;
      return armConfigurations.get(RobotSide.RIGHT) == AlexanderArmConfiguration.FOREARM;
   }
}

