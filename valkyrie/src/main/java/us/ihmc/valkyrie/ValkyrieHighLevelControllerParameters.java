package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final ValkyrieJointMap jointMap;
   private final ValkyrieStandPrepParameters standPrepParameters;

   private final boolean runningOnRealRobot;

   public ValkyrieHighLevelControllerParameters(boolean runningOnRealRobot, ValkyrieJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = jointMap;
      standPrepParameters = new ValkyrieStandPrepParameters(jointMap);
   }

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return standPrepParameters;
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
   {
      switch (state)
      {
      case WALKING:
         return getDesiredJointBehaviorForWalking();
      case DO_NOTHING_BEHAVIOR:
      case STAND_PREP_STATE:
      case STAND_READY:
      case STAND_TRANSITION_STATE:
      case CALIBRATION:
         return getDesiredJointBehaviorForHangingAround();
      default:
         throw new RuntimeException("Implement a desired joint behavior for the high level state " + state);
      }
   }

   private List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForWalking()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();
      if (runningOnRealRobot)
      {
         // Can go up to kp = 30.0, kd = 3.0
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, JointDesiredControlMode.EFFORT, 15.0, 1.5);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, JointDesiredControlMode.EFFORT, 15.0, 1.5);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, JointDesiredControlMode.EFFORT, 15.0, 1.5);
         // Can go up to kp = 30.0, kd = 4.0
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, JointDesiredControlMode.EFFORT, 15.0, 2.0);
         // Can go up to kp = 60.0, kd = 6.0
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, JointDesiredControlMode.EFFORT, 30.0, 3.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, JointDesiredControlMode.EFFORT, 30.0, 3.0);
         // Can go up to kp = 30.0, kd = 2.0
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, JointDesiredControlMode.EFFORT, 15.0, 1.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, JointDesiredControlMode.EFFORT, 15.0, 1.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, JointDesiredControlMode.EFFORT, 15.0, 1.0);
         // Can go up to kp = 30.0, kd = 2.0
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, JointDesiredControlMode.EFFORT, 15.0, 1.0);
         // Can go up to kp = 50.0, kd = 1.0
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, JointDesiredControlMode.EFFORT, 30.0, 1.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, JointDesiredControlMode.EFFORT, 30.0, 1.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_ROLL, JointDesiredControlMode.EFFORT, 30.0, 1.0);

         // position controlled on the real robot:
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_ROLL, JointDesiredControlMode.POSITION, 7.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, JointDesiredControlMode.POSITION, 20.0, 0.5);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.FIRST_WRIST_PITCH, JointDesiredControlMode.POSITION, 20.0, 0.5);

         configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, JointDesiredControlMode.POSITION, 0.0, 0.0);
         configureBehavior(behaviors, jointMap, NeckJointName.PROXIMAL_NECK_PITCH, JointDesiredControlMode.POSITION, 0.0, 0.0);
         configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, JointDesiredControlMode.POSITION, 0.0, 0.0);
      }
      else
      {
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, JointDesiredControlMode.EFFORT, 0.0, 0.0);

         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_ROLL, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureSymmetricBehavior(behaviors, jointMap, ArmJointName.FIRST_WRIST_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);

         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureBehavior(behaviors, jointMap, SpineJointName.SPINE_ROLL, JointDesiredControlMode.EFFORT, 0.0, 0.0);

         configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureBehavior(behaviors, jointMap, NeckJointName.PROXIMAL_NECK_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
         configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, JointDesiredControlMode.EFFORT, 0.0, 0.0);
      }

      return behaviors;
   }

   private List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForHangingAround()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();

      configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_PITCH, JointDesiredControlMode.EFFORT, 15.0, 1.5);
      configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_ROLL, JointDesiredControlMode.EFFORT, 15.0, 1.5);
      configureSymmetricBehavior(behaviors, jointMap, ArmJointName.SHOULDER_YAW, JointDesiredControlMode.EFFORT, 12.0, 1.2);
      configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_PITCH, JointDesiredControlMode.EFFORT, 12.0, 3.0);

      configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_YAW, JointDesiredControlMode.EFFORT, 30.0, 6.2);
      configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_ROLL, JointDesiredControlMode.EFFORT, 150.0, 7.5);
      configureSymmetricBehavior(behaviors, jointMap, LegJointName.HIP_PITCH, JointDesiredControlMode.EFFORT, 160.0, 7.0);
      configureSymmetricBehavior(behaviors, jointMap, LegJointName.KNEE_PITCH, JointDesiredControlMode.EFFORT, 75.0, 3.0);
      configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_PITCH, JointDesiredControlMode.EFFORT, 25.0, 2.0);
      configureSymmetricBehavior(behaviors, jointMap, LegJointName.ANKLE_ROLL, JointDesiredControlMode.EFFORT, 17.0, 1.0);

      configureBehavior(behaviors, jointMap, SpineJointName.SPINE_YAW, JointDesiredControlMode.EFFORT, 30.0, 3.0);
      configureBehavior(behaviors, jointMap, SpineJointName.SPINE_PITCH, JointDesiredControlMode.EFFORT, 180.0, 12.0);
      configureBehavior(behaviors, jointMap, SpineJointName.SPINE_ROLL, JointDesiredControlMode.EFFORT, 180.0, 12.0);

      // position controlled on the real robot:
      JointDesiredControlMode controlMode = runningOnRealRobot ? JointDesiredControlMode.POSITION : JointDesiredControlMode.EFFORT;

      configureSymmetricBehavior(behaviors, jointMap, ArmJointName.ELBOW_ROLL, controlMode, 15.0, 0.0);
      configureSymmetricBehavior(behaviors, jointMap, ArmJointName.WRIST_ROLL, controlMode, 6.0, 0.3);
      configureSymmetricBehavior(behaviors, jointMap, ArmJointName.FIRST_WRIST_PITCH, controlMode, 6.0, 0.3);

      configureBehavior(behaviors, jointMap, NeckJointName.PROXIMAL_NECK_PITCH, controlMode, 0.0, 0.0);
      configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_YAW, controlMode, 0.0, 0.0);
      configureBehavior(behaviors, jointMap, NeckJointName.DISTAL_NECK_PITCH, controlMode, 0.0, 0.0);

      return behaviors;
   }

   private static void configureBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors, DRCRobotJointMap jointMap,
                                         SpineJointName jointName, JointDesiredControlMode controlMode, double stiffness, double damping)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
      List<String> names = Collections.singletonList(jointMap.getSpineJointName(jointName));
      behaviors.add(new GroupParameter<>(jointName.toString(), jointBehavior, names));
   }

   private static void configureBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors, DRCRobotJointMap jointMap,
                                         NeckJointName jointName, JointDesiredControlMode controlMode, double stiffness, double damping)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
      List<String> names = Collections.singletonList(jointMap.getNeckJointName(jointName));
      behaviors.add(new GroupParameter<>(jointName.toString(), jointBehavior, names));
   }

   private static void configureSymmetricBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors,
                                                  DRCRobotJointMap jointMap, LegJointName jointName, JointDesiredControlMode controlMode, double stiffness,
                                                  double damping)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
      behaviors.add(new GroupParameter<>(jointName.toString(), jointBehavior, getLeftAndRightJointNames(jointMap, jointName)));
   }

   private static void configureSymmetricBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors,
                                                  DRCRobotJointMap jointMap, ArmJointName jointName, JointDesiredControlMode controlMode, double stiffness,
                                                  double damping)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
      behaviors.add(new GroupParameter<>(jointName.toString(), jointBehavior, getLeftAndRightJointNames(jointMap, jointName)));
   }

   private static List<String> getLeftAndRightJointNames(DRCRobotJointMap jointMap, LegJointName legJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(jointMap.getLegJointName(side, legJointName));
      }
      return jointNames;
   }

   private static List<String> getLeftAndRightJointNames(DRCRobotJointMap jointMap, ArmJointName armJointName)
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
      return runningOnRealRobot ? HighLevelControllerName.STAND_PREP_STATE : HighLevelControllerName.WALKING;
   }

   @Override
   public HighLevelControllerName getFallbackControllerState()
   {
      return runningOnRealRobot ? HighLevelControllerName.STAND_PREP_STATE : HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      return runningOnRealRobot;
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
      return 3.0;
   }

   @Override
   public double getCalibrationDuration()
   {
      return 10.0;
   }

   @Override
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersNoLoad()
   {
      List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> ret = new ArrayList<>();

      for (LegJointName legJointName : new LegJointName[]{LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL})
      { // Hip joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setAlphas(0.9992, 0.85);
         List<String> jointNames = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         ret.add(new GroupParameter<>(legJointName.getCamelCaseName(), parameters, jointNames));
      }

      for (LegJointName legJointName : new LegJointName[]{LegJointName.KNEE_PITCH, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL})
      { // Knee and ankle joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         List<String> jointNames = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         ret.add(new GroupParameter<>(legJointName.getCamelCaseName(), parameters, jointNames));
      }

      for (SpineJointName spineJointName : jointMap.getSpineJointNames())
      { // Spine joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setAlphas(0.9996, 0.85);
         List<String> jointNames = Collections.singletonList(jointMap.getSpineJointName(spineJointName));
         ret.add(new GroupParameter<>(spineJointName.getCamelCaseNameForStartOfExpression(), parameters, jointNames));
      }

      for (ArmJointName armJointName : new ArmJointName[]{ArmJointName.ELBOW_ROLL})
      { // Forearm elbow joint
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setAlphaPosition(0.9998);
         parameters.setAlphaVelocity(0.84);
         parameters.setMaxPositionError(0.2);
         parameters.setMaxVelocity(2.0);
         List<String> jointNames = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            jointNames.add(jointMap.getArmJointName(robotSide, armJointName));
         ret.add(new GroupParameter<>(armJointName.getCamelCaseNameForStartOfExpression(), parameters, jointNames));
      }

      for (ArmJointName armJointName : new ArmJointName[]{ArmJointName.FIRST_WRIST_PITCH, ArmJointName.WRIST_ROLL})
      { // Forearm wrist joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setAlphaPosition(0.9995);
         parameters.setAlphaVelocity(0.83);
         parameters.setMaxPositionError(0.2);
         parameters.setMaxVelocity(2.0);
         List<String> jointNames = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            jointNames.add(jointMap.getArmJointName(robotSide, armJointName));
         ret.add(new GroupParameter<>(armJointName.getCamelCaseNameForStartOfExpression(), parameters, jointNames));
      }

      for (NeckJointName neckJointName : jointMap.getNeckJointNames())
      { // Neck joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setAlphaPosition(0.9996);
         parameters.setAlphaVelocity(0.95);
         parameters.setMaxPositionError(0.2);
         parameters.setMaxVelocity(2.0);
         List<String> jointNames = Collections.singletonList(jointMap.getNeckJointName(neckJointName));
         ret.add(new GroupParameter<>(neckJointName.getCamelCaseName(), parameters, jointNames));
      }

      return ret;
   }
}
