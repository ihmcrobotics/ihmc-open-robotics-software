package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class FullRobotModelTestTools
{
   public static class RandomFullHumanoidRobotModel implements FullHumanoidRobotModel
   {
      private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private static final Vector3D roll = new Vector3D(1.0, 0.0, 0.0);
      private static final Vector3D pitch = new Vector3D(0.0, 1.0, 0.0);
      private static final Vector3D yaw = new Vector3D(0.0, 0.0, 1.0);

      private final RigidBodyBasics elevator;
      private final RigidBodyBasics pelvis;
      private final RigidBodyBasics chest;
      private final RigidBodyBasics head;

      private final SixDoFJoint rootJoint;

      private final LinkedHashMap<String, OneDoFJointBasics> oneDoFJoints = new LinkedHashMap<>();

      private final HashMap<SpineJointName, OneDoFJointBasics> spineJoints = new HashMap<>();
      private final HashMap<NeckJointName, OneDoFJointBasics> neckJoints = new HashMap<>();

      private final SideDependentList<HashMap<ArmJointName, OneDoFJointBasics>> armJoints = new SideDependentList<>();
      private final SideDependentList<HashMap<LegJointName, OneDoFJointBasics>> legJoints = new SideDependentList<>();
      private final SideDependentList<ArrayList<OneDoFJointBasics>> armJointIDsList = new SideDependentList<>();
      private final SideDependentList<ArrayList<OneDoFJointBasics>> legJointIDsList = new SideDependentList<>();

      private final LegJointName[] legJointNames;
      private final ArmJointName[] armJointNames;
      private final SpineJointName[] spineJointNames;
      private final NeckJointName[] neckJointNames;

      private final SideDependentList<HashMap<LimbName, RigidBodyBasics>> endEffectors = new SideDependentList<>();
      private final SideDependentList<RigidBodyBasics> hands = new SideDependentList<>();
      private final SideDependentList<RigidBodyBasics> feet = new SideDependentList<>();

      private final SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();

      private final double totalMass;

      private final IMUDefinition[] imuDefinitions = new IMUDefinition[0];
      private final ForceSensorDefinition[] forceSensorDefinitions = new ForceSensorDefinition[0];

      private final RobotSpecificJointNames robotSpecificJointNames;

      public RandomFullHumanoidRobotModel(Random random)
      {
         //ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
         elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());

         rootJoint = new SixDoFJoint("rootJoint", elevator);
         pelvis = MultiBodySystemRandomTools.nextRigidBody(random, "pelvis", rootJoint);

         addSpine(random);
         chest = MultiBodySystemRandomTools.nextRigidBody(random, "chest", spineJoints.get(SpineJointName.SPINE_YAW));

         addNeck(random);
         head = MultiBodySystemRandomTools.nextRigidBody(random, "head", neckJoints.get(NeckJointName.DISTAL_NECK_PITCH));

         for (RobotSide robotSide : RobotSide.values)
         {
            endEffectors.put(robotSide, new HashMap<>());

            armJointIDsList.put(robotSide, new ArrayList<>());
            armJoints.put(robotSide, new HashMap<>());
            addArm(robotSide, random);

            legJointIDsList.put(robotSide, new ArrayList<>());
            legJoints.put(robotSide, new HashMap<>());
            addLeg(robotSide, random);

            RigidBodyTransform soleToAnkleTransform = new RigidBodyTransform();
            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            MovingReferenceFrame soleFrame = MovingReferenceFrame.constructFrameFixedInParent(sidePrefix + "Sole", getEndEffectorFrame(robotSide, LimbName.LEG), soleToAnkleTransform);
            soleFrames.put(robotSide, soleFrame);
         }

         totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

         legJointNames = new LegJointName[legJoints.get(RobotSide.LEFT).size()];
         armJointNames = new ArmJointName[armJoints.get(RobotSide.LEFT).size()];
         spineJointNames = new SpineJointName[spineJoints.size()];
         neckJointNames = new NeckJointName[neckJoints.size()];

         legJoints.get(RobotSide.LEFT).keySet().toArray(legJointNames);
         armJoints.get(RobotSide.LEFT).keySet().toArray(armJointNames);
         spineJoints.keySet().toArray(spineJointNames);
         neckJoints.keySet().toArray(neckJointNames);

         robotSpecificJointNames = new RobotSpecificJointNames()
         {
            @Override
            public LegJointName[] getLegJointNames()
            {
               return legJointNames;
            }

            @Override
            public ArmJointName[] getArmJointNames()
            {
               return armJointNames;
            }

            @Override
            public SpineJointName[] getSpineJointNames()
            {
               return spineJointNames;
            }

            @Override
            public NeckJointName[] getNeckJointNames()
            {
               return neckJointNames;
            }
         };
      }

      private void addSpine(Random random)
      {
         RevoluteJoint spineRoll = MultiBodySystemRandomTools.nextRevoluteJoint(random, "spineRoll", roll, pelvis);
         RigidBodyBasics trunnion1 = MultiBodySystemRandomTools.nextRigidBody(random, "spineTrunnion1", spineRoll);

         RevoluteJoint spinePitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, "spinePitch", pitch, trunnion1);
         RigidBodyBasics trunnion2 = MultiBodySystemRandomTools.nextRigidBody(random, "spineTrunnion2", spinePitch);

         RevoluteJoint spineYaw = MultiBodySystemRandomTools.nextRevoluteJoint(random, "spineYaw", yaw, trunnion2);

         spineJoints.put(SpineJointName.SPINE_ROLL, spineRoll);
         spineJoints.put(SpineJointName.SPINE_PITCH, spinePitch);
         spineJoints.put(SpineJointName.SPINE_YAW, spineYaw);

         oneDoFJoints.put(spineRoll.getName(), spineRoll);
         oneDoFJoints.put(spinePitch.getName(), spinePitch);
         oneDoFJoints.put(spineYaw.getName(), spineYaw);
      }

      private void addNeck(Random random)
      {
         RevoluteJoint lowerNeckPitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, "lowerNeckPitch", pitch, chest);
         RigidBodyBasics trunnion1 = MultiBodySystemRandomTools.nextRigidBody(random, "neckTrunnion1", lowerNeckPitch);

         RevoluteJoint neckYaw = MultiBodySystemRandomTools.nextRevoluteJoint(random, "neckYaw", yaw, trunnion1);
         RigidBodyBasics trunnion2 = MultiBodySystemRandomTools.nextRigidBody(random, "neckTrunnion2", neckYaw);

         RevoluteJoint upperNeckPitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, "upperNeckPitch", pitch, trunnion2);

         neckJoints.put(NeckJointName.PROXIMAL_NECK_PITCH, lowerNeckPitch);
         neckJoints.put(NeckJointName.DISTAL_NECK_YAW, neckYaw);
         neckJoints.put(NeckJointName.DISTAL_NECK_PITCH, upperNeckPitch);

         oneDoFJoints.put(lowerNeckPitch.getName(), lowerNeckPitch);
         oneDoFJoints.put(neckYaw.getName(), neckYaw);
         oneDoFJoints.put(upperNeckPitch.getName(), upperNeckPitch);
      }

      private void addArm(RobotSide robotSide, Random random)
      {
         String prefix = robotSide.getShortLowerCaseName();

         RevoluteJoint shoulderYaw = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_shoulderYaw", yaw, chest);
         RigidBodyBasics shoulderTrunnion = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_shoulderTrunnion", shoulderYaw);

         RevoluteJoint shoulderRoll = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_shoulderRoll", roll, shoulderTrunnion);
         RigidBodyBasics shoulder = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_shoulder", shoulderRoll);

         RevoluteJoint shoulderPitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_shoulderPitch", pitch, shoulder);
         RigidBodyBasics upperArm = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_upperArm", shoulderPitch);

         RevoluteJoint elbowPitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_elbowPitch", pitch, upperArm);
         RigidBodyBasics lowerArm = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_lowerArm", elbowPitch);

         RevoluteJoint elbowYaw = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_elbowYaw", yaw, lowerArm);
         RigidBodyBasics forearm = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_forearm", elbowYaw);

         RevoluteJoint firstWristPitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_firstWristPitch", pitch, forearm);
         RigidBodyBasics wristTrunnion1 = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_wristTrunnion1", firstWristPitch);

         RevoluteJoint wristRoll = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_wristRoll", roll, wristTrunnion1);
         RigidBodyBasics hand = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_hand", wristRoll);

         armJoints.get(robotSide).put(ArmJointName.SHOULDER_YAW, shoulderYaw);
         armJoints.get(robotSide).put(ArmJointName.SHOULDER_ROLL, shoulderRoll);
         armJoints.get(robotSide).put(ArmJointName.SHOULDER_PITCH, shoulderPitch);
         armJoints.get(robotSide).put(ArmJointName.ELBOW_PITCH, elbowPitch);
         armJoints.get(robotSide).put(ArmJointName.ELBOW_YAW, elbowYaw);
         armJoints.get(robotSide).put(ArmJointName.FIRST_WRIST_PITCH, firstWristPitch);
         armJoints.get(robotSide).put(ArmJointName.WRIST_ROLL, wristRoll);

         armJointIDsList.get(robotSide).add(shoulderYaw);
         armJointIDsList.get(robotSide).add(shoulderRoll);
         armJointIDsList.get(robotSide).add(shoulderPitch);
         armJointIDsList.get(robotSide).add(elbowPitch);
         armJointIDsList.get(robotSide).add(elbowYaw);
         armJointIDsList.get(robotSide).add(firstWristPitch);
         armJointIDsList.get(robotSide).add(wristRoll);

         hands.put(robotSide, hand);
         endEffectors.get(robotSide).put(LimbName.ARM, hand);

         oneDoFJoints.put(shoulderYaw.getName(), shoulderYaw);
         oneDoFJoints.put(shoulderRoll.getName(), shoulderRoll);
         oneDoFJoints.put(shoulderPitch.getName(), shoulderPitch);
         oneDoFJoints.put(elbowPitch.getName(), elbowPitch);
         oneDoFJoints.put(elbowYaw.getName(), elbowYaw);
         oneDoFJoints.put(firstWristPitch.getName(), firstWristPitch);
         oneDoFJoints.put(wristRoll.getName(), wristRoll);
      }

      private void addLeg(RobotSide robotSide, Random random)
      {
         String prefix = robotSide.getShortLowerCaseName();

         RevoluteJoint hipYaw = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_hipYaw", yaw, pelvis);
         RigidBodyBasics hipTrunnion = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_hipTrunnion", hipYaw);

         RevoluteJoint hipRoll = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_hipRoll", roll, hipTrunnion);
         RigidBodyBasics hip = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_hip", hipRoll);

         RevoluteJoint hipPitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_hipPitch", pitch, hip);
         RigidBodyBasics upperLeg = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_upperLeg", hipPitch);

         RevoluteJoint kneePitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_kneePitch", pitch, upperLeg);
         RigidBodyBasics lowerLeg = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_lowerLeg", kneePitch);

         RevoluteJoint ankleRoll = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_ankleRoll", roll, lowerLeg);
         RigidBodyBasics ankleTrunnion = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_ankleTrunnion", ankleRoll);

         RevoluteJoint anklePitch = MultiBodySystemRandomTools.nextRevoluteJoint(random, prefix + "_anklePitch", pitch, ankleTrunnion);
         RigidBodyBasics foot = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "_foot", anklePitch);

         legJoints.get(robotSide).put(LegJointName.HIP_YAW, hipYaw);
         legJoints.get(robotSide).put(LegJointName.HIP_ROLL, hipRoll);
         legJoints.get(robotSide).put(LegJointName.HIP_PITCH, hipPitch);
         legJoints.get(robotSide).put(LegJointName.KNEE_PITCH, kneePitch);
         legJoints.get(robotSide).put(LegJointName.ANKLE_ROLL, ankleRoll);
         legJoints.get(robotSide).put(LegJointName.ANKLE_PITCH, anklePitch);

         legJointIDsList.get(robotSide).add(hipYaw);
         legJointIDsList.get(robotSide).add(hipRoll);
         legJointIDsList.get(robotSide).add(hipPitch);
         legJointIDsList.get(robotSide).add(kneePitch);
         legJointIDsList.get(robotSide).add(ankleRoll);
         legJointIDsList.get(robotSide).add(anklePitch);

         feet.put(robotSide, foot);
         endEffectors.get(robotSide).put(LimbName.LEG, foot);

         oneDoFJoints.put(hipYaw.getName(), hipYaw);
         oneDoFJoints.put(hipRoll.getName(), hipRoll);
         oneDoFJoints.put(hipPitch.getName(), hipPitch);
         oneDoFJoints.put(kneePitch.getName(), kneePitch);
         oneDoFJoints.put(ankleRoll.getName(), ankleRoll);
         oneDoFJoints.put(anklePitch.getName(), anklePitch);
      }

      @Override public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return robotSpecificJointNames;
      }

      @Override public void updateFrames()
      {
         elevator.updateFramesRecursively();
      }

      @Override public MovingReferenceFrame getElevatorFrame()
      {
         return elevator.getBodyFixedFrame();
      }

      @Override public FloatingJointBasics getRootJoint()
      {
         return rootJoint;
      }

      @Override public RigidBodyBasics getElevator()
      {
         return elevator;
      }

      @Override public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
      {
         return spineJoints.get(spineJointName);
      }

      @Override public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
      {
         return neckJoints.get(neckJointName);
      }

      @Override public JointBasics getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }

      @Override public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override public RigidBodyBasics getPelvis()
      {
         return pelvis;
      }

      @Override public RigidBodyBasics getRootBody()
      {
         return pelvis;
      }

      @Override public RigidBodyBasics getChest()
      {
         return chest;
      }

      @Override public RigidBodyBasics getHead()
      {
         return head;
      }

      @Override public ReferenceFrame getHeadBaseFrame()
      {
         return head.getParentJoint().getFrameAfterJoint();
      }

      @Override public OneDoFJointBasics[] getOneDoFJoints()
      {
         OneDoFJointBasics[] oneDoFJointsAsArray = new OneDoFJointBasics[oneDoFJoints.size()];
         oneDoFJoints.values().toArray(oneDoFJointsAsArray);
         return oneDoFJointsAsArray;
      }

      @Override public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
      {
         return oneDoFJoints;
      }

      @Override public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
      {
         oneDoFJointsToPack.clear();
         JointBasics parent = oneDoFJointAtEndOfChain;

         while (parent != rootJoint)
         {
            if (parent instanceof OneDoFJointBasics)
            {
               oneDoFJointsToPack.add((OneDoFJointBasics) parent);
            }

            parent = parent.getPredecessor().getParentJoint();
         }

         Collections.reverse(oneDoFJointsToPack);
      }

      @Override public OneDoFJointBasics[] getControllableOneDoFJoints()
      {
         return getOneDoFJoints();
      }

      @Override public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
         Collection<OneDoFJointBasics> values = oneDoFJoints.values();
         oneDoFJointsToPack.addAll(values);
      }

      @Override public OneDoFJointBasics getOneDoFJointByName(String name)
      {
         return oneDoFJoints.get(name);
      }

      @Override public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
         getOneDoFJoints(oneDoFJointsToPack);
      }

      @Override public IMUDefinition[] getIMUDefinitions()
      {
         return imuDefinitions;
      }

      @Override public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return forceSensorDefinitions;
      }

      @Override public double getTotalMass()
      {
         return totalMass;
      }

      @Override public MovingReferenceFrame getFrameAfterLegJoint(RobotSide robotSide, LegJointName legJointName)
      {
         return legJoints.get(robotSide).get(legJointName).getFrameAfterJoint();
      }

      @Override public OneDoFJointBasics getLegJoint(RobotSide robotSide, LegJointName legJointName)
      {
         return legJoints.get(robotSide).get(legJointName);
      }

      @Override public OneDoFJointBasics getArmJoint(RobotSide robotSide, ArmJointName armJointName)
      {
         return armJoints.get(robotSide).get(armJointName);
      }

      @Override public RigidBodyBasics getFoot(RobotSide robotSide)
      {
         return feet.get(robotSide);
      }

      @Override public RigidBodyBasics getHand(RobotSide robotSide)
      {
         return hands.get(robotSide);
      }

      @Override public RigidBodyBasics getEndEffector(RobotSide robotSide, LimbName limbName)
      {
         return endEffectors.get(robotSide).get(limbName);
      }

      @Override public MovingReferenceFrame getEndEffectorFrame(RobotSide robotSide, LimbName limbName)
      {
         return endEffectors.get(robotSide).get(limbName).getBodyFixedFrame();
      }

      @Override public MovingReferenceFrame getHandControlFrame(RobotSide robotSide)
      {
         return hands.get(robotSide).getBodyFixedFrame();
      }

      @Override public MovingReferenceFrame getSoleFrame(RobotSide robotSide)
      {
         return soleFrames.get(robotSide);
      }

      @Override public SideDependentList<MovingReferenceFrame> getSoleFrames()
      {
         return soleFrames;
      }

      @Override public void setJointAngles(RobotSide side, LimbName limb, double[] q)
      {
         int i = 0;
         if (limb == LimbName.ARM)
         {
            for (OneDoFJointBasics joint : armJointIDsList.get(side))
            {
               joint.setQ(q[i]);
               i++;
            }
         }
         else if (limb == LimbName.LEG)
         {
            for (OneDoFJointBasics jnt : legJointIDsList.get(side))
            {
               jnt.setQ(q[i]);
               i++;
            }
         }

      }

      @Override
      public RobotSide[] getRobotSegments()
      {
         return RobotSide.values;
      }
   }
}
