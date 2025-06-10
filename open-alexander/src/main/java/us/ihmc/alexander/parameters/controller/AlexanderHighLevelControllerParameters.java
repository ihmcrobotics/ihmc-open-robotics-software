package us.ihmc.alexander.parameters.controller;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import us.ihmc.alexander.AlexanderJointMap;
import us.ihmc.alexander.AlexanderVersionInterface;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointVelocityIntegratorResetMode;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.yoVariables.filters.AlphaFilterTools;

import static us.ihmc.alexander.parameters.controller.HighLevelParametersTools.*;
import static us.ihmc.sensorProcessing.outputData.JointDesiredControlMode.EFFORT;
import static us.ihmc.sensorProcessing.outputData.JointDesiredControlMode.POSITION;

public class AlexanderHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final AlexanderVersionInterface alexanderVersion;
   private final AlexanderJointMap jointMap;
   private final RobotTarget target;
   private final AlexanderStandPrepSetPoints standPrepSetPoints;

   public AlexanderHighLevelControllerParameters(AlexanderVersionInterface alexanderVersion, AlexanderJointMap jointMap, RobotTarget target)
   {
      this.alexanderVersion = alexanderVersion;
      this.jointMap = jointMap;
      this.target = target;
      standPrepSetPoints = new AlexanderStandPrepSetPoints(jointMap);
   }

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return standPrepSetPoints;
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
   {
      switch (state)
      {
         case WALKING:
            return getDesiredJointBehaviorForWalkingNotLoaded();
         case DO_NOTHING_BEHAVIOR:
            return getDesiredJointBehaviorForDoNothing();
         case STAND_PREP_STATE:
         case STAND_READY:
         case STAND_TRANSITION_STATE:
         case EXIT_WALKING:
         case FREEZE_STATE:
            return getDesiredJointBehaviorForHangingAround();
         default:
            throw new RuntimeException("Implement a desired joint behavior for the high level state " + state);
      }
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorsUnderLoad(HighLevelControllerName state)
   {
      if (state == HighLevelControllerName.WALKING)
         return getDesiredJointBehaviorForWalkingUnderLoad();
      else
         return null;
   }

   private List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForDoNothing()
   {
      if (target != RobotTarget.SCS)
         return getDesiredJointBehaviorForHangingAround();

      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      List<String> allJoint = new ArrayList<String>();
      allJoint.addAll(jointMap.getSpineJointNamesAsStrings());
      allJoint.addAll(jointMap.getNeckJointNamesAsStrings());
      allJoint.addAll(jointMap.getArmJointNamesAsStrings());
      allJoint.addAll(jointMap.getLegJointNamesAsStrings());
      behaviors.add(new GroupParameter<JointDesiredBehaviorReadOnly>("wholeBody", new JointDesiredBehavior(EFFORT), allJoint));

      return behaviors;
   }

   private List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForWalkingUnderLoad()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      if (target == RobotTarget.REAL_ROBOT)
      {
         double maxPosError = 0.15;
         double maxVelError = 1.00;
         double velScale = 1.0;

         double upperBodyStiffness = 5.0;
         double upperBodyDamping = 8.0;

         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, 0.0, 3.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);

         if (alexanderVersion.hasArm(RobotSide.LEFT) || alexanderVersion.hasArm(RobotSide.RIGHT))
         {
            // Default parameters
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, upperBodyStiffness, upperBodyDamping, maxPosError, maxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, upperBodyStiffness, upperBodyDamping, maxPosError, maxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, upperBodyStiffness, upperBodyDamping, maxPosError, maxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, upperBodyStiffness, upperBodyDamping, maxPosError, maxVelError, velScale);

            if (alexanderVersion.hasCycloidForearms())
            {
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, EFFORT, upperBodyStiffness, upperBodyDamping, maxPosError, maxVelError, velScale);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, EFFORT, upperBodyStiffness, upperBodyDamping, maxPosError, maxVelError, velScale);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, EFFORT, upperBodyStiffness, upperBodyDamping, maxPosError, maxVelError, velScale);
            }
         }
      }
      else
      {
         JointDesiredBehavior allJointBehaviors = new JointDesiredBehavior(EFFORT, 0.0, 0.0);
         List<String> allJoints = Arrays.asList(jointMap.getOrderedJointNames());
         behaviors.add(new GroupParameter<>("", allJointBehaviors, allJoints));
      }

      return behaviors;
   }

   private List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForWalkingNotLoaded()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      if (target == RobotTarget.REAL_ROBOT)
      {
         double legStiffness = 0.0;
         double legDamping = 0.0;
         double maxPosError = 0.15;
         double maxVelError = 1.00;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, legStiffness, legDamping, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, legStiffness, legDamping, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, legStiffness, legDamping, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, legStiffness, legDamping, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, legStiffness, legDamping, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, legStiffness, legDamping, maxPosError, maxVelError, velScale);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);

         if (alexanderVersion.hasHead())
         {
            configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, EFFORT, 100.0, 6.0);
            configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, EFFORT, 100.0, 6.0);
         }

         if (alexanderVersion.hasArm(RobotSide.LEFT) || alexanderVersion.hasArm(RobotSide.RIGHT))
         {
            // Default parameters
            double upperArmMaxPosError = 0.35;
            double upperArmMaxVelError = 2.0;
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, 3.0, 7.5, upperArmMaxPosError, upperArmMaxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 3.0, 4.0, upperArmMaxPosError, upperArmMaxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 3.0, 4.0, upperArmMaxPosError, upperArmMaxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 2.5, 4.0, upperArmMaxPosError, upperArmMaxVelError, velScale);

            if (alexanderVersion.hasCycloidForearms())
            { // Cycloid forearms
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, POSITION, 3.5, 4.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, POSITION, 2.0, 3.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, POSITION, 2.0, 5.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            }
         }
      }
      else
      {
         JointDesiredBehavior allJointBehaviors = new JointDesiredBehavior(EFFORT, 0.0, 0.0);
         List<String> allJoints = Arrays.asList(jointMap.getOrderedJointNames());
         behaviors.add(new GroupParameter<>("", allJointBehaviors, allJoints));
      }

      return behaviors;
   }

   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForHangingAround()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      if (target == RobotTarget.REAL_ROBOT)
      {
         double maxPosError = 0.4;
         double maxVelError = 2.0;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, 175.0, 10.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, 80.0, 2.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, 250.0, 3.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, 250.0, 5.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, 250.0, 5.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, 40.0, 5.0, maxPosError, maxVelError, velScale);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, EFFORT, 50.0, 5.0, maxPosError, maxVelError, velScale);

         double smallStiffness = 10.0;
         double smallDamping = 0.5;
         if (alexanderVersion.hasHead())
         {
            configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, EFFORT, smallStiffness, smallDamping);
            configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, EFFORT, smallStiffness, smallDamping);
         }

         if (alexanderVersion.hasArm(RobotSide.LEFT) || alexanderVersion.hasArm(RobotSide.RIGHT))
         {
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, 80.0, 8.0, maxPosError, maxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 100.0, 9.0, maxPosError, maxVelError, velScale);
            // TODO Cogging model for the right shoulder z (J3) needs to be redone, velocity looks like crap can't crank the damping as much.
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 60.0, 3.0, maxPosError, maxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 100.0, 4.0, maxPosError, maxVelError, velScale);

            if (alexanderVersion.hasCycloidForearms())
            { // Cycloid forearms
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, EFFORT, smallStiffness, smallDamping, maxPosError, maxVelError, velScale);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, EFFORT, smallStiffness, smallDamping, maxPosError, maxVelError, velScale);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, EFFORT, smallStiffness, smallDamping, maxPosError, maxVelError, velScale);
            }
         }
      }
      else
      {
         double maxPosError = 0.2;
         double maxVelError = 2.0;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, 125.0, 10.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, 50.0, 5.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, 250.0, 8.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, 250.0, 8.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, 100.0, 5.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, 40.0, 5.0, maxPosError, maxVelError, velScale);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, EFFORT, 50.0, 5.0, maxPosError, maxVelError, velScale);

         double smallStiffness = 10.0;
         double smallDamping = 0.5;
         if (alexanderVersion.hasHead())
         {
            configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, EFFORT, smallStiffness, smallDamping);
            configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, EFFORT, smallStiffness, smallDamping);
         }

         if (alexanderVersion.hasArm(RobotSide.LEFT) || alexanderVersion.hasArm(RobotSide.RIGHT))
         {
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, 80.0, 8.0, maxPosError, maxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 100.0, 9.0, maxPosError, maxVelError, velScale);
            // TODO Cogging model for the right shoulder z (J3) needs to be redone, velocity looks like crap can't crank the damping as much.
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 60.0, 3.0, maxPosError, maxVelError, velScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 100.0, 4.0, maxPosError, maxVelError, velScale);

            if (alexanderVersion.hasCycloidForearms())
            { // Cycloid forearms
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, EFFORT, smallStiffness, smallDamping, maxPosError, maxVelError, velScale);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, EFFORT, smallStiffness, smallDamping, maxPosError, maxVelError, velScale);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, EFFORT, smallStiffness, smallDamping, maxPosError, maxVelError, velScale);
            }
         }
      }

      return behaviors;
   }


   @Override
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParameters(HighLevelControllerName state)
   {
      switch (state)
      {
         case WALKING:
            return getJointAccelerationIntegrationParametersForWalkingNotLoaded();
         case DO_NOTHING_BEHAVIOR:
         case STAND_PREP_STATE:
         case STAND_READY:
         case STAND_TRANSITION_STATE:
         case EXIT_WALKING:
         case FREEZE_STATE:
            return getJointAccelerationIntegrationParametersForHangingAround();
         default:
            throw new RuntimeException("Implement a desired joint behavior for the high level state " + state);
      }
   }

   @Override
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersUnderLoad(HighLevelControllerName state)
   {
      if (state == HighLevelControllerName.WALKING)
         return getJointAccelerationIntegrationParametersForWalkingUnderLoad();
      else
         return null;
   }

   private List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForWalkingNotLoaded()
   {
      List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> ret = new ArrayList<>();

      { // Pelvis yaw joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(0.75);
         parameters.setPositionBreakFrequency(Double.POSITIVE_INFINITY);
         parameters.setMaxPositionError(0.0); // Cancel integration into position
         parameters.setVelocityReferenceAlpha(0.0); // Initially cancel integration into velocity
         List<String> jointNames = new ArrayList<>();

         for (LegJointName legJointName : new LegJointName[] {LegJointName.HIP_YAW})
         { // Hip Yaw joints
            for (RobotSide robotSide : RobotSide.values)
               jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         }

         { // Spine Yaw joint
            jointNames.add(jointMap.getSpineJointName(SpineJointName.SPINE_YAW));
         }

         ret.add(new GroupParameter<>("PelvisYaws", parameters, jointNames));
      }

      { // Leg joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(1.0);
         parameters.setPositionBreakFrequency(Double.POSITIVE_INFINITY);
         parameters.setMaxPositionError(0.0); // Cancel integration into position
         parameters.setVelocityReferenceAlpha(0.0); // initially cancel integration into velocity
         List<String> jointNames = new ArrayList<>();

         for (LegJointName legJointName : new LegJointName[] {LegJointName.HIP_PITCH,
                                                              LegJointName.HIP_ROLL,
                                                              LegJointName.KNEE_PITCH,
                                                              LegJointName.ANKLE_PITCH,
                                                              LegJointName.ANKLE_ROLL})
         {
            for (RobotSide robotSide : RobotSide.values)
               jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         }

         ret.add(new GroupParameter<>("LegJoints", parameters, jointNames));
      }

      { // Neck joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(1.00);
         parameters.setPositionBreakFrequency(0.05);
         parameters.setMaxPositionError(0.10);
         parameters.setVelocityReferenceAlpha(1.0);
         List<String> jointNames = new ArrayList<>();

         jointNames.add(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH));
         jointNames.add(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW));

         ret.add(new GroupParameter<>("Neck", parameters, jointNames));
      }

      if (alexanderVersion.hasArm(RobotSide.LEFT) || alexanderVersion.hasArm(RobotSide.RIGHT)) // Arm joints
      {
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(1.0);
         parameters.setPositionBreakFrequency(0.1);
         parameters.setMaxPositionError(0.45);
         parameters.setMaxVelocityError(2.0);
         parameters.setVelocityReferenceAlpha(0.7);
         List<String> jointNames = new ArrayList<>();



         for (ArmJointName armJointName : new ArmJointName[] {ArmJointName.SHOULDER_PITCH,
                                                              ArmJointName.SHOULDER_ROLL,
                                                              ArmJointName.SHOULDER_YAW,
                                                              ArmJointName.ELBOW_PITCH})
         {
            for (RobotSide robotSide : RobotSide.values)
               jointNames.add(jointMap.getArmJointName(robotSide, armJointName));
         }

         if (alexanderVersion.hasCycloidForearms())
         {
            for (ArmJointName armJointName : new ArmJointName[] {ArmJointName.ELBOW_YAW,
                                                                 ArmJointName.WRIST_ROLL,
                                                                 ArmJointName.WRIST_YAW})
            {
               for (RobotSide robotSide : RobotSide.values)
                  jointNames.add(jointMap.getArmJointName(robotSide, armJointName));
            }
         }

         ret.add(new GroupParameter<>("ArmJoints", parameters, jointNames));
      }

      for (NeckJointName neckJointName : jointMap.getNeckJointNames())
      { // Neck joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setPositionBreakFrequency(AlphaFilterTools.computeBreakFrequencyGivenAlpha(0.9996, 0.004));
         parameters.setVelocityBreakFrequency(AlphaFilterTools.computeBreakFrequencyGivenAlpha(0.95, 0.004));
         parameters.setMaxPositionError(0.2);
         parameters.setMaxVelocityError(2.0);
         List<String> jointNames = Collections.singletonList(jointMap.getNeckJointName(neckJointName));
         ret.add(new GroupParameter<>(neckJointName.getCamelCaseName(), parameters, jointNames));
      }

      return ret;
   }

   private List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForWalkingUnderLoad()
   {
      List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> ret = new ArrayList<>();

      { // Pelvis yaw joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(0.75);
         parameters.setPositionBreakFrequency(Double.POSITIVE_INFINITY);
         parameters.setMaxPositionError(0.0); // Cancel integration into position
         parameters.setVelocityReferenceAlpha(1.0);
         List<String> jointNames = new ArrayList<>();

         for (LegJointName legJointName : new LegJointName[] {LegJointName.HIP_YAW})
         { // Hip Yaw joints
            for (RobotSide robotSide : RobotSide.values)
               jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         }

         { // Spine Yaw joint
            jointNames.add(jointMap.getSpineJointName(SpineJointName.SPINE_YAW));
         }

         ret.add(new GroupParameter<>("PelvisYaws", parameters, jointNames));
      }

      { // Hip Pitch & Roll joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(3.0);
         parameters.setPositionBreakFrequency(Double.POSITIVE_INFINITY);
         parameters.setMaxPositionError(0.0); // Cancel integration into position
         parameters.setVelocityReferenceAlpha(0.0);
         List<String> jointNames = new ArrayList<>();

         for (LegJointName legJointName : new LegJointName[] {LegJointName.HIP_PITCH, LegJointName.HIP_ROLL})
         {
            for (RobotSide robotSide : RobotSide.values)
               jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         }

         ret.add(new GroupParameter<>("HipPitchRoll", parameters, jointNames));
      }

      for (LegJointName legJointName : new LegJointName[] {LegJointName.KNEE_PITCH, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL})
      { // Knee and ankle joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(3.0);
         parameters.setPositionBreakFrequency(Double.POSITIVE_INFINITY);
         parameters.setMaxPositionError(0.0); // Cancel integration into position
         parameters.setVelocityReferenceAlpha(0.0);
         parameters.setVelocityResetMode(JointVelocityIntegratorResetMode.ZERO_VELOCITY);
         List<String> jointNames = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         ret.add(new GroupParameter<>(legJointName.getCamelCaseName(), parameters, jointNames));
      }

      return ret;
   }

   private List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForHangingAround()
   {
      // Possible add a single parameter that is shared between all joints here.
      return null;
   }

   @Override
   public HighLevelControllerName getDefaultInitialControllerState()
   {
      boolean shouldUseStandPrep = (target == RobotTarget.REAL_ROBOT);
      return shouldUseStandPrep ? HighLevelControllerName.FREEZE_STATE : HighLevelControllerName.WALKING;
   }

   @Override
   public HighLevelControllerName getFallbackControllerState()
   {
      boolean shouldUseFreezeState = (target == RobotTarget.REAL_ROBOT);
      return shouldUseFreezeState ? HighLevelControllerName.FREEZE_STATE : HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      // TODO (20250608 rgriffin) when we're confident on the feet being loaded and everything, we can change this to true
      return false;
   }

   @Override
   public double getTimeToMoveInStandPrep()
   {
      // TODO (20250608 rgriffin) we can decrease this, 10 seconds is really slow.
      return 10.0;
   }

   @Override
   public double getMinimumTimeInStandReady()
   {
      return 3.0;
   }

   @Override
   public double getTimeInStandTransition()
   {
      return 1.5;
   }
   @Override
   public double getTimeInStandTransition(HighLevelControllerName controllerName)
   {
      if (controllerName == HighLevelControllerName.EXIT_WALKING)
      {
         return 5.0;
      }
      else
      {
         return getTimeInStandTransition();
      }
   }

   @Override
   public double getCalibrationDuration()
   {
      return Double.NaN;
   }

   @Override
   public double getCalibrationMaxTorqueOffset()
   {
      return Double.NaN;
   }
}
