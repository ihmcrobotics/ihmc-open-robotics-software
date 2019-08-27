package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public class AtlasHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final AtlasJointMap jointMap;
   private boolean runningOnRealRobot;

   public AtlasHighLevelControllerParameters(boolean runningOnRealRobot, AtlasJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = jointMap;
   }

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return null;
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
   {
      switch (state)
      {
      case WALKING:
         return getDesiredJointBehaviorForWalking();
      case DO_NOTHING_BEHAVIOR:
      case FALLING_STATE:
         return getDesiredJointBehaviorForDoNothing();
      default:
         throw new RuntimeException("Implement a desired joint behavior for the high level state " + state);
      }
   }

   private List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForWalking()
   {
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();
      JointDesiredControlMode positionControlMode = runningOnRealRobot ? JointDesiredControlMode.POSITION : JointDesiredControlMode.EFFORT;

      // neck
      JointDesiredBehavior neckJointBehavior = new JointDesiredBehavior(positionControlMode);
      behaviors.add(new GroupParameter<>("Neck", neckJointBehavior, jointMap.getNeckJointNamesAsStrings()));

      // arms
      JointDesiredBehavior armJointBehavior = new JointDesiredBehavior(positionControlMode);
      behaviors.add(new GroupParameter<>("Arms", armJointBehavior, jointMap.getArmJointNamesAsStrings()));

      // spine
      JointDesiredBehavior spineJointBehavior = new JointDesiredBehavior(JointDesiredControlMode.EFFORT);
      behaviors.add(new GroupParameter<>("Spine", spineJointBehavior, jointMap.getSpineJointNamesAsStrings()));

      // legs
      JointDesiredBehavior legJointBehavior = new JointDesiredBehavior(JointDesiredControlMode.EFFORT);
      behaviors.add(new GroupParameter<>("Legs", legJointBehavior, jointMap.getLegJointNamesAsStrings()));

      return behaviors;
   }

   private List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForDoNothing()
   {
      List<String> allJoints = Arrays.asList(jointMap.getOrderedJointNames());
      JointDesiredBehavior allJointBehaviors = new JointDesiredBehavior(JointDesiredControlMode.EFFORT);

      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();
      behaviors.add(new GroupParameter<>("", allJointBehaviors, allJoints));
      return behaviors;
   }

   @Override
   public HighLevelControllerName getDefaultInitialControllerState()
   {
      return runningOnRealRobot ? HighLevelControllerName.DO_NOTHING_BEHAVIOR : HighLevelControllerName.WALKING;
   }

   @Override
   public HighLevelControllerName getFallbackControllerState()
   {
      return HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      return false;
   }

   @Override
   public double getTimeToMoveInStandPrep()
   {
      return 0;
   }

   @Override
   public double getMinimumTimeInStandReady()
   {
      return 0;
   }

   @Override
   public double getTimeInStandTransition()
   {
      return 0;
   }

   @Override
   public double getCalibrationDuration()
   {
      return 0;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParameters(HighLevelControllerName state)
   {
      List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> integrationSettings = new ArrayList<>();

      // Neck joints:
      JointAccelerationIntegrationParameters neckJointSettings = new JointAccelerationIntegrationParameters();
      neckJointSettings.setPositionBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.9996, 0.004));
      neckJointSettings.setVelocityBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.95, 0.004));
      neckJointSettings.setMaxPositionError(0.2);
      neckJointSettings.setMaxVelocity(2.0);

      List<String> neckJointNames = new ArrayList<>();
      for (NeckJointName name : jointMap.getNeckJointNames())
         neckJointNames.add(jointMap.getNeckJointName(name));
      integrationSettings.add(new GroupParameter<>("NeckAccelerationIntegration", neckJointSettings, neckJointNames));

      // Shoulder joints:
      JointAccelerationIntegrationParameters shoulderJointSettings = new JointAccelerationIntegrationParameters();
      shoulderJointSettings.setPositionBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.9998, 0.004));
      shoulderJointSettings.setVelocityBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.95, 0.004));
      shoulderJointSettings.setMaxPositionError(0.2);
      shoulderJointSettings.setMaxVelocity(2.0);

      ArmJointName[] shoulderJoints = new ArmJointName[]{ArmJointName.SHOULDER_YAW, ArmJointName.SHOULDER_ROLL};
      List<String> shoulderJointNames = new ArrayList<>();
      for (ArmJointName name : shoulderJoints)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            shoulderJointNames.add(jointMap.getArmJointName(robotSide, name));
         }
      }
      integrationSettings.add(new GroupParameter<>("ShoulderAccelerationIntegration", shoulderJointSettings, shoulderJointNames));

      // Elbow joints:
      JointAccelerationIntegrationParameters elbowJointSettings = new JointAccelerationIntegrationParameters();
      elbowJointSettings.setPositionBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.9996, 0.004));
      elbowJointSettings.setVelocityBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.95, 0.004));
      elbowJointSettings.setMaxPositionError(0.2);
      elbowJointSettings.setMaxVelocity(2.0);

      ArmJointName[] elbowJoints = new ArmJointName[]{ArmJointName.ELBOW_PITCH, ArmJointName.ELBOW_ROLL};
      List<String> elbowJointNames = new ArrayList<>();
      for (ArmJointName name : elbowJoints)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            elbowJointNames.add(jointMap.getArmJointName(robotSide, name));
         }
      }
      integrationSettings.add(new GroupParameter<>("ElbowAccelerationIntegration", elbowJointSettings, elbowJointNames));

      // Wrist joints:
      JointAccelerationIntegrationParameters wristJointSettings = new JointAccelerationIntegrationParameters();
      wristJointSettings.setPositionBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.9999, 0.004));
      wristJointSettings.setVelocityBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.95, 0.004));
      wristJointSettings.setMaxPositionError(0.2);
      wristJointSettings.setMaxVelocity(2.0);

      ArmJointName[] wristJoints = new ArmJointName[]{ArmJointName.FIRST_WRIST_PITCH, ArmJointName.WRIST_ROLL, ArmJointName.SECOND_WRIST_PITCH};
      List<String> wristJointNames = new ArrayList<>();
      for (ArmJointName name : wristJoints)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            wristJointNames.add(jointMap.getArmJointName(robotSide, name));
         }
      }
      integrationSettings.add(new GroupParameter<>("WristAccelerationIntegration", wristJointSettings, wristJointNames));

      return integrationSettings;
   }
}
