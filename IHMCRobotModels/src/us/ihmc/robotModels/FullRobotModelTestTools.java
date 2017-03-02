package us.ihmc.robotModels;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

import java.util.*;

public class FullRobotModelTestTools
{
   public static class RandomFullHumanoidRobotModel implements FullHumanoidRobotModel
   {
      private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private static final Vector3D roll = new Vector3D(1.0, 0.0, 0.0);
      private static final Vector3D pitch = new Vector3D(0.0, 1.0, 0.0);
      private static final Vector3D yaw = new Vector3D(0.0, 0.0, 1.0);

      private final RigidBody elevator;
      private final RigidBody pelvis;
      private final RigidBody chest;
      private final RigidBody head;

      private final SixDoFJoint rootJoint;

      private final LinkedHashMap<String, OneDoFJoint> oneDoFJoints = new LinkedHashMap<>();

      private final HashMap<SpineJointName, OneDoFJoint> spineJoints = new HashMap<>();
      private final HashMap<NeckJointName, OneDoFJoint> neckJoints = new HashMap<>();

      private final SideDependentList<HashMap<ArmJointName, OneDoFJoint>> armJoints = new SideDependentList<>();
      private final SideDependentList<HashMap<LegJointName, OneDoFJoint>> legJoints = new SideDependentList<>();
      private final SideDependentList<ArrayList<OneDoFJoint>> armJointIDsList = new SideDependentList<>();
      private final SideDependentList<ArrayList<OneDoFJoint>> legJointIDsList = new SideDependentList<>();

      private final LegJointName[] legJointNames;
      private final ArmJointName[] armJointNames;
      private final SpineJointName[] spineJointNames;
      private final NeckJointName[] neckJointNames;

      private final SideDependentList<HashMap<LimbName, RigidBody>> endEffectors = new SideDependentList<>();
      private final SideDependentList<RigidBody> hands = new SideDependentList<>();
      private final SideDependentList<RigidBody> feet = new SideDependentList<>();

      private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

      private final double totalMass;

      private final IMUDefinition[] imuDefinitions = new IMUDefinition[0];
      private final ForceSensorDefinition[] forceSensorDefinitions = new ForceSensorDefinition[0];
      private final ContactSensorDefinition[] contactSensorDefinitions = new ContactSensorDefinition[0];

      private final RobotSpecificJointNames robotSpecificJointNames;

      public RandomFullHumanoidRobotModel(Random random)
      {
         ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
         elevator = new RigidBody("elevator", elevatorFrame);

         rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
         pelvis = ScrewTestTools.addRandomRigidBody("pelvis", random, rootJoint);

         addSpine(random);
         chest = ScrewTestTools.addRandomRigidBody("chest", random, spineJoints.get(SpineJointName.SPINE_YAW));

         addNeck(random);
         head = ScrewTestTools.addRandomRigidBody("head", random, neckJoints.get(NeckJointName.DISTAL_NECK_PITCH));

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
            ReferenceFrame soleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(sidePrefix + "Sole", getEndEffectorFrame(robotSide, LimbName.LEG), soleToAnkleTransform);
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
         RevoluteJoint spineRoll = ScrewTestTools.addRandomRevoluteJoint("spineRoll", roll, random, pelvis);
         RigidBody trunnion1 = ScrewTestTools.addRandomRigidBody("spineTrunnion1", random, spineRoll);

         RevoluteJoint spinePitch = ScrewTestTools.addRandomRevoluteJoint("spinePitch", pitch, random, trunnion1);
         RigidBody trunnion2 = ScrewTestTools.addRandomRigidBody("spineTrunnion2", random, spinePitch);

         RevoluteJoint spineYaw = ScrewTestTools.addRandomRevoluteJoint("spineYaw", yaw, random, trunnion2);

         spineJoints.put(SpineJointName.SPINE_ROLL, spineRoll);
         spineJoints.put(SpineJointName.SPINE_PITCH, spinePitch);
         spineJoints.put(SpineJointName.SPINE_YAW, spineYaw);

         oneDoFJoints.put(spineRoll.getName(), spineRoll);
         oneDoFJoints.put(spinePitch.getName(), spinePitch);
         oneDoFJoints.put(spineYaw.getName(), spineYaw);
      }

      private void addNeck(Random random)
      {
         RevoluteJoint lowerNeckPitch = ScrewTestTools.addRandomRevoluteJoint("lowerNeckPitch", pitch, random, chest);
         RigidBody trunnion1 = ScrewTestTools.addRandomRigidBody("neckTrunnion1", random, lowerNeckPitch);

         RevoluteJoint neckYaw = ScrewTestTools.addRandomRevoluteJoint("neckYaw", yaw, random, trunnion1);
         RigidBody trunnion2 = ScrewTestTools.addRandomRigidBody("neckTrunnion2", random, neckYaw);

         RevoluteJoint upperNeckPitch = ScrewTestTools.addRandomRevoluteJoint("upperNeckPitch", pitch, random, trunnion2);

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

         RevoluteJoint shoulderYaw = ScrewTestTools.addRandomRevoluteJoint(prefix + "_shoulderYaw", yaw, random, chest);
         RigidBody shoulderTrunnion = ScrewTestTools.addRandomRigidBody(prefix + "_shoulderTrunnion", random, shoulderYaw);

         RevoluteJoint shoulderRoll = ScrewTestTools.addRandomRevoluteJoint(prefix + "_shoulderRoll", roll, random, shoulderTrunnion);
         RigidBody shoulder = ScrewTestTools.addRandomRigidBody(prefix + "_shoulder", random, shoulderRoll);

         RevoluteJoint shoulderPitch = ScrewTestTools.addRandomRevoluteJoint(prefix + "_shoulderPitch", pitch, random, shoulder);
         RigidBody upperArm = ScrewTestTools.addRandomRigidBody(prefix + "_upperArm", random, shoulderPitch);

         RevoluteJoint elbowPitch = ScrewTestTools.addRandomRevoluteJoint(prefix + "_elbowPitch", pitch, random, upperArm);
         RigidBody lowerArm = ScrewTestTools.addRandomRigidBody(prefix + "_lowerArm", random, elbowPitch);

         RevoluteJoint elbowYaw = ScrewTestTools.addRandomRevoluteJoint(prefix + "_elbowYaw", yaw, random, lowerArm);
         RigidBody forearm = ScrewTestTools.addRandomRigidBody(prefix + "_forearm", random, elbowYaw);

         RevoluteJoint firstWristPitch = ScrewTestTools.addRandomRevoluteJoint(prefix + "_firstWristPitch", pitch, random, forearm);
         RigidBody wristTrunnion1 = ScrewTestTools.addRandomRigidBody(prefix + "_wristTrunnion1", random, firstWristPitch);

         RevoluteJoint wristRoll = ScrewTestTools.addRandomRevoluteJoint(prefix + "_wristRoll", roll, random, wristTrunnion1);
         RigidBody hand = ScrewTestTools.addRandomRigidBody(prefix + "_hand", random, wristRoll);

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

         RevoluteJoint hipYaw = ScrewTestTools.addRandomRevoluteJoint(prefix + "_hipYaw", yaw, random, pelvis);
         RigidBody hipTrunnion = ScrewTestTools.addRandomRigidBody(prefix + "_hipTrunnion", random, hipYaw);

         RevoluteJoint hipRoll = ScrewTestTools.addRandomRevoluteJoint(prefix + "_hipRoll", roll, random, hipTrunnion);
         RigidBody hip = ScrewTestTools.addRandomRigidBody(prefix + "_hip", random, hipRoll);

         RevoluteJoint hipPitch = ScrewTestTools.addRandomRevoluteJoint(prefix + "_hipPitch", pitch, random, hip);
         RigidBody upperLeg = ScrewTestTools.addRandomRigidBody(prefix + "_upperLeg", random, hipPitch);

         RevoluteJoint kneePitch = ScrewTestTools.addRandomRevoluteJoint(prefix + "_kneePitch", pitch, random, upperLeg);
         RigidBody lowerLeg = ScrewTestTools.addRandomRigidBody(prefix + "_lowerLeg", random, kneePitch);

         RevoluteJoint ankleRoll = ScrewTestTools.addRandomRevoluteJoint(prefix + "_ankleRoll", roll, random, lowerLeg);
         RigidBody ankleTrunnion = ScrewTestTools.addRandomRigidBody(prefix + "_ankleTrunnion", random, ankleRoll);

         RevoluteJoint anklePitch = ScrewTestTools.addRandomRevoluteJoint(prefix + "_anklePitch", pitch, random, ankleTrunnion);
         RigidBody foot = ScrewTestTools.addRandomRigidBody(prefix + "_foot", random, anklePitch);

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

      @Override public ReferenceFrame getWorldFrame()
      {
         return worldFrame;
      }

      @Override public ReferenceFrame getElevatorFrame()
      {
         return elevator.getBodyFixedFrame();
      }

      @Override public FloatingInverseDynamicsJoint getRootJoint()
      {
         return rootJoint;
      }

      @Override public RigidBody getElevator()
      {
         return elevator;
      }

      @Override public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return spineJoints.get(spineJointName);
      }

      @Override public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return neckJoints.get(neckJointName);
      }

      @Override public InverseDynamicsJoint getLidarJoint(String lidarName)
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

      @Override public RigidBody getPelvis()
      {
         return pelvis;
      }

      @Override public RigidBody getChest()
      {
         return chest;
      }

      @Override public RigidBody getHead()
      {
         return head;
      }

      @Override public ReferenceFrame getHeadBaseFrame()
      {
         return head.getParentJoint().getFrameAfterJoint();
      }

      @Override public OneDoFJoint[] getOneDoFJoints()
      {
         OneDoFJoint[] oneDoFJointsAsArray = new OneDoFJoint[oneDoFJoints.size()];
         oneDoFJoints.values().toArray(oneDoFJointsAsArray);
         return oneDoFJointsAsArray;
      }

      @Override public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
      {
         return oneDoFJoints;
      }

      @Override public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
         oneDoFJointsToPack.clear();
         InverseDynamicsJoint parent = oneDoFJointAtEndOfChain;

         while (parent != rootJoint)
         {
            if (parent instanceof OneDoFJoint)
            {
               oneDoFJointsToPack.add((OneDoFJoint) parent);
            }

            parent = parent.getPredecessor().getParentJoint();
         }

         Collections.reverse(oneDoFJointsToPack);
      }

      @Override public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return getOneDoFJoints();
      }

      @Override public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
         Collection<OneDoFJoint> values = oneDoFJoints.values();
         oneDoFJointsToPack.addAll(values);
      }

      @Override public OneDoFJoint getOneDoFJointByName(String name)
      {
         return oneDoFJoints.get(name);
      }

      @Override public void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
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

      @Override public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return contactSensorDefinitions;
      }

      @Override public double getTotalMass()
      {
         return totalMass;
      }

      @Override public ReferenceFrame getFrameAfterLegJoint(RobotSide robotSide, LegJointName legJointName)
      {
         return legJoints.get(robotSide).get(legJointName).getFrameAfterJoint();
      }

      @Override public OneDoFJoint getLegJoint(RobotSide robotSide, LegJointName legJointName)
      {
         return legJoints.get(robotSide).get(legJointName);
      }

      @Override public OneDoFJoint getArmJoint(RobotSide robotSide, ArmJointName armJointName)
      {
         return armJoints.get(robotSide).get(armJointName);
      }

      @Override public RigidBody getFoot(RobotSide robotSide)
      {
         return feet.get(robotSide);
      }

      @Override public RigidBody getHand(RobotSide robotSide)
      {
         return hands.get(robotSide);
      }

      @Override public RigidBody getEndEffector(RobotSide robotSide, LimbName limbName)
      {
         return endEffectors.get(robotSide).get(limbName);
      }

      @Override public ReferenceFrame getEndEffectorFrame(RobotSide robotSide, LimbName limbName)
      {
         return endEffectors.get(robotSide).get(limbName).getBodyFixedFrame();
      }

      @Override public ReferenceFrame getHandControlFrame(RobotSide robotSide)
      {
         return hands.get(robotSide).getBodyFixedFrame();
      }

      @Override public FramePoint getStaticWristToFingerOffset(RobotSide robotSide, FingerName fingerName)
      {
         return null;
      }

      @Override public ReferenceFrame getSoleFrame(RobotSide robotSide)
      {
         return soleFrames.get(robotSide);
      }

      @Override public SideDependentList<ReferenceFrame> getSoleFrames()
      {
         return soleFrames;
      }

      @Override public void setJointAngles(RobotSide side, LimbName limb, double[] q)
      {
         int i = 0;
         if (limb == LimbName.ARM)
         {
            for (OneDoFJoint joint : armJointIDsList.get(side))
            {
               joint.setQ(q[i]);
               i++;
            }
         }
         else if (limb == LimbName.LEG)
         {
            for (OneDoFJoint jnt : legJointIDsList.get(side))
            {
               jnt.setQ(q[i]);
               i++;
            }
         }

      }
   }
}
