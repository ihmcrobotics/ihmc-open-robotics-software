package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.commons.lang3.tuple.ImmutableTriple;

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
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final ValkyrieJointMap jointMap;
   private final ValkyrieStandPrepParameters standPrepParameters;
   private final ValkyrieWalkingPositionControlParameters walkingParameters;

   private final boolean runningOnRealRobot;
   private final Set<String> positionControlledJoints = new HashSet<>();

   public ValkyrieHighLevelControllerParameters(boolean runningOnRealRobot, ValkyrieJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = jointMap;

      standPrepParameters = new ValkyrieStandPrepParameters(jointMap);
      walkingParameters = new ValkyrieWalkingPositionControlParameters(runningOnRealRobot, jointMap);

      positionControlledJoints.add(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH));
      positionControlledJoints.add(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH));
      positionControlledJoints.add(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW));
      positionControlledJoints.add(jointMap.getArmJointName(RobotSide.LEFT, ArmJointName.ELBOW_ROLL));
      positionControlledJoints.add(jointMap.getArmJointName(RobotSide.LEFT, ArmJointName.WRIST_ROLL));
      positionControlledJoints.add(jointMap.getArmJointName(RobotSide.LEFT, ArmJointName.FIRST_WRIST_PITCH));
      positionControlledJoints.add(jointMap.getArmJointName(RobotSide.RIGHT, ArmJointName.ELBOW_ROLL));
      positionControlledJoints.add(jointMap.getArmJointName(RobotSide.RIGHT, ArmJointName.WRIST_ROLL));
      positionControlledJoints.add(jointMap.getArmJointName(RobotSide.RIGHT, ArmJointName.FIRST_WRIST_PITCH));
   }

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return standPrepParameters;
   }

   @Override
   public JointDesiredControlMode getJointDesiredControlMode(String jointName, HighLevelControllerName state)
   {
      if (runningOnRealRobot)
      {
         if (positionControlledJoints.contains(jointName))
            return JointDesiredControlMode.POSITION;
      }

      return JointDesiredControlMode.EFFORT;
   }

   @Override
   public double getDesiredJointStiffness(String jointName, HighLevelControllerName state)
   {
      if (state != HighLevelControllerName.WALKING)
         return standPrepParameters.getProportionalGain(jointName);
      else
         return walkingParameters.getProportionalGain(jointName);
   }

   @Override
   public double getDesiredJointDamping(String jointName, HighLevelControllerName state)
   {
      if (state != HighLevelControllerName.WALKING)
         return standPrepParameters.getDerivativeGain(jointName);
      else
         return walkingParameters.getDerivativeGain(jointName);
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
   public List<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> getJointAccelerationIntegrationParametersNoLoad()
   {
      List<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> ret = new ArrayList<>();

      for (LegJointName legJointName : new LegJointName[]{LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL})
      { // Hip joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setAlphas(0.9992, 0.85);
         List<String> jointNames = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         ret.add(new ImmutableTriple<>(legJointName.getCamelCaseName(), parameters, jointNames));
      }

      for (LegJointName legJointName : new LegJointName[]{LegJointName.KNEE_PITCH, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL})
      { // Knee and ankle joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         List<String> jointNames = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            jointNames.add(jointMap.getLegJointName(robotSide, legJointName));
         ret.add(new ImmutableTriple<>(legJointName.getCamelCaseName(), parameters, jointNames));
      }

      for (SpineJointName spineJointName : jointMap.getSpineJointNames())
      { // Spine joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setAlphas(0.9996, 0.85);
         List<String> jointNames = Collections.singletonList(jointMap.getSpineJointName(spineJointName));
         ret.add(new ImmutableTriple<>(spineJointName.getCamelCaseNameForStartOfExpression(), parameters, jointNames));
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
         ret.add(new ImmutableTriple<>(armJointName.getCamelCaseNameForStartOfExpression(), parameters, jointNames));
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
         ret.add(new ImmutableTriple<>(armJointName.getCamelCaseNameForStartOfExpression(), parameters, jointNames));
      }

      for (NeckJointName neckJointName : jointMap.getNeckJointNames())
      { // Neck joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setAlphaPosition(0.9996);
         parameters.setAlphaVelocity(0.95);
         parameters.setMaxPositionError(0.2);
         parameters.setMaxVelocity(2.0);
         List<String> jointNames = Collections.singletonList(jointMap.getNeckJointName(neckJointName));
         ret.add(new ImmutableTriple<>(neckJointName.getCamelCaseName(), parameters, jointNames));
      }

      return ret;
   }
}
