package us.ihmc.valkyrie;

import java.util.HashSet;
import java.util.Set;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final ValkyrieStandPrepParameters standPrepParameters;
   private final ValkyrieWalkingPositionControlParameters walkingParameters;

   private final boolean runningOnRealRobot;
   private final Set<String> positionControlledJoints = new HashSet<>();

   public ValkyrieHighLevelControllerParameters(boolean runningOnRealRobot, ValkyrieJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      standPrepParameters = new ValkyrieStandPrepParameters(jointMap);
      walkingParameters = new ValkyrieWalkingPositionControlParameters(jointMap);

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
   public JointDesiredControlMode getJointDesiredControlMode(String jointName, HighLevelController state)
   {
      if (runningOnRealRobot)
      {
         if (positionControlledJoints.contains(jointName))
            return JointDesiredControlMode.POSITION;
      }

      return JointDesiredControlMode.EFFORT;
   }

   @Override
   public double getDesiredJointStiffness(String jointName, HighLevelController state)
   {
      if (state != HighLevelController.WALKING)
         return standPrepParameters.getProportionalGain(jointName);
      else
         return 0.0;
   }

   @Override
   public double getDesiredJointDamping(String jointName, HighLevelController state)
   {
      if (state != HighLevelController.WALKING)
         return standPrepParameters.getDerivativeGain(jointName);
      else
         return 0.0;
   }

   @Override
   public HighLevelController getDefaultInitialControllerState()
   {
      return runningOnRealRobot ? HighLevelController.STAND_PREP_STATE : HighLevelController.WALKING;
   }

   @Override
   public HighLevelController getFallbackControllerState()
   {
      return runningOnRealRobot ? HighLevelController.STAND_READY : HighLevelController.DO_NOTHING_BEHAVIOR;
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
}
