package us.ihmc.openAlexander.parameters.controller;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import us.ihmc.openAlexander.AlexanderJointMap;
import us.ihmc.openAlexander.AlexanderVersionInterface;
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
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;

import static us.ihmc.sensorProcessing.outputData.JointDesiredControlMode.EFFORT;
import static us.ihmc.sensorProcessing.outputData.JointDesiredControlMode.POSITION;
import static us.ihmc.wholeBodyController.parameters.HighLevelParametersTools.*;

public class OpenAlexanderHighLevelControllerParameters implements HighLevelControllerParameters
{
   protected final AlexanderVersionInterface alexanderVersion;
   protected final AlexanderJointMap jointMap;
   protected final RobotTarget target;
   private final AlexanderStandPrepSetPoints standPrepSetPoints;

   public OpenAlexanderHighLevelControllerParameters(AlexanderVersionInterface alexanderVersion, AlexanderJointMap jointMap, RobotTarget target)
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
         case CUSTOM1:
         case QUICKSTER:
            return getDesiredJointBehaviorForWalkingNotLoaded();
         case DO_NOTHING_BEHAVIOR:
            return getDesiredJointBehaviorForDoNothing();
         case STAND_PREP_STATE:
         case STAND_READY:
         case STAND_TRANSITION_STATE:
         case EXIT_WALKING:
         case FREEZE_STATE:
            return getDesiredJointBehaviorForHangingAround();
         case CALIBRATION:
            return getDesiredJointBehaviorForCalibration();
         default:
            throw new RuntimeException("Implement a desired joint behavior for the high level state " + state);
      }
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorsUnderLoad(HighLevelControllerName state)
   {
      if (state == HighLevelControllerName.WALKING || state == HighLevelControllerName.CUSTOM1 || state == HighLevelControllerName.QUICKSTER)
         return getDesiredJointBehaviorForWalkingUnderLoad();
      else
         return null;
   }

   protected List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForDoNothing()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      List<String> allJoint = new ArrayList<String>();
      allJoint.addAll(jointMap.getSpineJointNamesAsStrings());
      allJoint.addAll(jointMap.getNeckJointNamesAsStrings());
      allJoint.addAll(jointMap.getArmJointNamesAsStrings());
      allJoint.addAll(jointMap.getLegJointNamesAsStrings());
      JointDesiredBehavior jointDesiredBehavior = new JointDesiredBehavior(EFFORT);
      jointDesiredBehavior.setMaxPositionError(0.0);
      jointDesiredBehavior.setMaxVelocityError(0.0);
      behaviors.add(new GroupParameter<JointDesiredBehaviorReadOnly>("wholeBody", jointDesiredBehavior, allJoint));

      return behaviors;
   }

   protected List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForWalkingUnderLoad()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      if (target == RobotTarget.REAL_ROBOT)
      {
         double maxPosError = 0.15;
         double maxVelError = 1.00;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);

         { // Cycloid arms
            double cycloidVelScale = 1.0;

            // Default parameters
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, 5.0, 8.0, 0.25, 1.0, cycloidVelScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 5.0, 8.0, 0.25, 1.0, cycloidVelScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 5.0, 8.0, 0.25, 1.0, cycloidVelScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 5.0, 8.0, 0.25, 1.0, cycloidVelScale);

            if (alexanderVersion.hasCycloidForearm())
            {
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, EFFORT, 5.0, 8.0, 0.25, 1.0, cycloidVelScale);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, EFFORT, 5.0, 8.0, 0.25, 1.0, cycloidVelScale);
               configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, EFFORT, 5.0, 8.0, 0.25, 1.0, cycloidVelScale);
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

   protected List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForWalkingNotLoaded()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      if (target == RobotTarget.REAL_ROBOT)
      {
         double maxPosError = 0.15;
         double maxVelError = 1.00;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, 0.0, 10.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, 0.0, 10.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, 0.0, 15.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, 0.0, 15.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, 0.0, 10.0, maxPosError, maxVelError, velScale);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, EFFORT, 0.0, 0.0, maxPosError, maxVelError, velScale);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, EFFORT, 100.0, 6.0, maxPosError, maxVelError, velScale);

         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, EFFORT, 100.0, 6.0);
         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, EFFORT, 100.0, 6.0);

         { // Cycloid upper arms
            double cycloidVelScale = 1.0;

            // Default parameters
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, 3.0, 7.5, 0.35, 2.0, cycloidVelScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 3.0, 4.0, 0.35, 2.0, cycloidVelScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 3.0, 4.0, 0.35, 2.0, cycloidVelScale);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 2.5, 4.0, 0.35, 2.0, cycloidVelScale);
         }

         if (alexanderVersion.hasCycloidForearm())
         { // Cycloid forearms
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, POSITION, 3.5, 4.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, POSITION, 2.0, 3.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, POSITION, 2.0, 5.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
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
         double maxPosError = 0.15;
         double maxVelError = 1.00;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, 200.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, 1000.0, 3.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, 1000.0, 3.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, 1000.0, 20.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, 800.0, 3.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, 800.0, 3.0, maxPosError, maxVelError, velScale);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, EFFORT, 101.0, 0.0, maxPosError, maxVelError, velScale);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, EFFORT, 1000.0, 3.0, maxPosError, maxVelError, velScale);

         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, EFFORT, 1000.0, 3.0);
         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, EFFORT, 1000.0, 3.0);

         double maxArmPosError = 2.00;
         double maxArmVelError = 1.00;
         double armVelScale = 1.0;

         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, 80.0, 8.0, maxArmPosError, maxArmVelError, armVelScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 100.0, 9.0, maxArmPosError, maxArmVelError, armVelScale);
         // TODO Cogging model fir the right shoulder z (J3) needs to be redone, velocity looks like crap can't crank the damping as much.
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 60.0, 3.0, maxArmPosError, maxArmVelError, armVelScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 100.0, 4.0, maxArmPosError, maxArmVelError, armVelScale);

         if (alexanderVersion.hasCycloidForearm())
         { // Cycloid forearms
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, POSITION, 20.0, 4.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, POSITION, 15.0, 2.5, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, POSITION, 12.0, 2.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
         }
      }
      else
      {
         double maxPosError = 1.5;
         double maxVelError = 0.10;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, POSITION, 500.0, 50.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, POSITION, 500.0, 50.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, POSITION, 500.0, 50.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, POSITION, 500.0, 50.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, POSITION, 650.0, 2.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, POSITION, 200.0, 2.0, maxPosError, maxVelError, velScale);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, POSITION, 1500.0, 150.0, maxPosError, maxVelError, velScale);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, POSITION, 1500.0, 150.0, maxPosError, maxVelError, velScale);

         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, POSITION, 1500.0, 150.0);
         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, POSITION, 1500.0, 150.0);

         double maxArmPosError = 2.00;
         double maxArmVelError = 1.00;
         double armVelScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, POSITION, 120.0, 12.0, maxArmPosError, maxArmVelError, armVelScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, POSITION, 120.0, 12.0, maxArmPosError, maxArmVelError, armVelScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, POSITION, 120.0, 12.0, maxArmPosError, maxArmVelError, armVelScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, POSITION, 120.0, 12.0, maxArmPosError, maxArmVelError, armVelScale);

         if (alexanderVersion.hasCycloidForearm())
         { // Cycloid forearms
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, POSITION, 20.0, 4.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, POSITION, 15.0, 2.5, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, POSITION, 12.0, 2.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
         }
      }

      return behaviors;
   }

   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForCalibration()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      if (target == RobotTarget.REAL_ROBOT)
      {
         double maxPosError = 0.15;
         double maxVelError = 1.00;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, EFFORT, 200.0, 0.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, EFFORT, 500.0, 2.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, EFFORT, 500.0, 2.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, EFFORT, 500.0, 2.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, EFFORT, 500.0, 2.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, EFFORT, 500.0, 2.0, maxPosError, maxVelError, velScale);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, EFFORT, 100.0, 0.0, maxPosError, maxVelError, velScale);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, EFFORT, 500.0, 0.0, maxPosError, maxVelError, velScale);

         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, EFFORT, 500.0, 0.0);
         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, EFFORT, 500.0, 0.0);

         double maxArmPosError = 2.00;
         double maxArmVelError = 1.00;
         double armVelScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, 25.0, 2.5, maxArmPosError, maxArmVelError, armVelScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 25.0, 2.5, maxArmPosError, maxArmVelError, armVelScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 15.0, 2.0, maxArmPosError, maxArmVelError, armVelScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 20.0, 2.0, maxArmPosError, maxArmVelError, armVelScale);

         if (alexanderVersion.hasCycloidForearm())
         { // Cycloid forearms
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, POSITION, 15.0, 3.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, POSITION, 12.0, 2.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, POSITION, 10.0, 2.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
         }
      }
      else
      {
         double maxPosError = 0.15;
         double maxVelError = 0.10;
         double velScale = 1.0;
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, POSITION, 500.0, 50.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, POSITION, 500.0, 50.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, POSITION, 500.0, 50.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, POSITION, 500.0, 50.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, POSITION, 40.0, 2.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, POSITION, 40.0, 2.0, maxPosError, maxVelError, velScale);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, POSITION, 1500.0, 150.0, maxPosError, maxVelError, velScale);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, POSITION, 1500.0, 150.0, maxPosError, maxVelError, velScale);

         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, POSITION, 1500.0, 150.0);
         configureNeckBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, POSITION, 1500.0, 150.0);

         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, EFFORT, 120.0, 12.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, EFFORT, 120.0, 12.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, EFFORT, 120.0, 12.0, maxPosError, maxVelError, velScale);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, EFFORT, 120.0, 12.0, maxPosError, maxVelError, velScale);

         if (alexanderVersion.hasCycloidForearm())
         { // Cycloid forearms
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_YAW, POSITION, 15.0, 3.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, POSITION, 12.0, 2.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
            configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_YAW, POSITION, 10.0, 2.0, Double.MAX_VALUE, Double.MAX_VALUE, 1.0);
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
         case CUSTOM1:
         case QUICKSTER:
            return getJointAccelerationIntegrationParametersForFastWalkingNotLoaded();
         case DO_NOTHING_BEHAVIOR:
         case STAND_PREP_STATE:
         case STAND_READY:
         case STAND_TRANSITION_STATE:
         case EXIT_WALKING:
         case CALIBRATION:
         case FREEZE_STATE:
            return getJointAccelerationIntegrationParametersForHangingAround();
         default:
            throw new RuntimeException("Implement a desired joint behavior for the high level state " + state);
      }
   }

   @Override
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersUnderLoad(HighLevelControllerName state)
   {
      if (state == HighLevelControllerName.WALKING || state == HighLevelControllerName.CUSTOM1 || state == HighLevelControllerName.QUICKSTER)
         return getJointAccelerationIntegrationParametersForWalkingUnderLoad();
      else
         return null;
   }

   protected List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForWalkingNotLoaded()
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

      { // Leg joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(1.0);
         parameters.setPositionBreakFrequency(Double.POSITIVE_INFINITY);
         parameters.setMaxPositionError(0.0); // Cancel integration into position
         parameters.setVelocityReferenceAlpha(1.0);
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

      { // Arm joints
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

         if (alexanderVersion.hasCycloidForearm())
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
         parameters.setPositionBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.9996, 0.004));
         parameters.setVelocityBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.95, 0.004));
         parameters.setMaxPositionError(0.2);
         parameters.setMaxVelocityError(2.0);
         List<String> jointNames = Collections.singletonList(jointMap.getNeckJointName(neckJointName));
         ret.add(new GroupParameter<>(neckJointName.getCamelCaseName(), parameters, jointNames));
      }

      return ret;
   }

   protected List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForFastWalkingNotLoaded()
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

      { // Leg hydraulic joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(3.0);
         parameters.setPositionBreakFrequency(Double.POSITIVE_INFINITY);
         parameters.setMaxPositionError(0.0); // Cancel integration into position
         parameters.setVelocityReferenceAlpha(1.0);
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

      { // Spine pitch and roll joints
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

      { // Upper arm joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setVelocityBreakFrequency(1.8);
         parameters.setPositionBreakFrequency(0.025);
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

         if (alexanderVersion.hasCycloidForearm())
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

   protected List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForWalkingUnderLoad()
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

   protected List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForHangingAround()
   {
      // Possible add a single parameter that is shared between all joints here.
      return null;
   }

   public static List<String> getLeftAndRightJointNames(HumanoidJointNameMap jointMap, LegJointName legJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(jointMap.getLegJointName(side, legJointName));
      }
      return jointNames;
   }

   public static List<String> getLeftAndRightJointNames(HumanoidJointNameMap jointMap, ArmJointName armJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(jointMap.getArmJointName(side, armJointName));
      }
      return jointNames;
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
      return true;
   }

   @Override
   public double getTimeToMoveInStandPrep()
   {
      return 5.0;
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
      return 30.0;
   }

   @Override
   public double getCalibrationMaxTorqueOffset()
   {
      return 15.0;
   }
}
