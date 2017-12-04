package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
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
   public JointDesiredControlMode getJointDesiredControlMode(String joint, HighLevelControllerName state)
   {
      switch (state)
      {
      case WALKING:
         return getJointDesiredControlModeWalking(joint);
      default:
         return JointDesiredControlMode.EFFORT;
      }
   }

   private JointDesiredControlMode getJointDesiredControlModeWalking(String joint)
   {
      if (runningOnRealRobot)
      {
         for (NeckJointName name : jointMap.getNeckJointNames())
         {
            if (jointMap.getNeckJointName(name).equals(joint))
            {
               return JointDesiredControlMode.POSITION;
            }
         }

         for (RobotSide robotSide : RobotSide.values)
         {
            for (ArmJointName name : jointMap.getArmJointNames())
            {
               if (jointMap.getArmJointName(robotSide, name).equals(joint))
               {
                  return JointDesiredControlMode.POSITION;
               }
            }
         }
      }

      return JointDesiredControlMode.EFFORT;
   }

   @Override
   public double getDesiredJointStiffness(String joint, HighLevelControllerName state)
   {
      return 0.0;
   }

   @Override
   public double getDesiredJointDamping(String joint, HighLevelControllerName state)
   {
      return 0.0;
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
   public List<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> getJointAccelerationIntegrationParametersNoLoad()
   {
      ArrayList<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> integrationSettings = new ArrayList<>();

      // Neck joints:
      JointAccelerationIntegrationParameters neckJointSettings = new JointAccelerationIntegrationParameters();
      neckJointSettings.setAlphaPosition(0.9996);
      neckJointSettings.setAlphaVelocity(0.95);
      neckJointSettings.setMaxPositionError(0.2);
      neckJointSettings.setMaxVelocity(2.0);

      List<String> neckJointNames = new ArrayList<>();
      for (NeckJointName name : jointMap.getNeckJointNames())
         neckJointNames.add(jointMap.getNeckJointName(name));
      integrationSettings.add(new ImmutableTriple<>("NeckAccelerationIntegration", neckJointSettings, neckJointNames));

      // Shoulder joints:
      JointAccelerationIntegrationParameters shoulderJointSettings = new JointAccelerationIntegrationParameters();
      shoulderJointSettings.setAlphaPosition(0.9998);
      shoulderJointSettings.setAlphaVelocity(0.95);
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
      integrationSettings.add(new ImmutableTriple<>("ShoulderAccelerationIntegration", shoulderJointSettings, shoulderJointNames));

      // Elbow joints:
      JointAccelerationIntegrationParameters elbowJointSettings = new JointAccelerationIntegrationParameters();
      elbowJointSettings.setAlphaPosition(0.9996);
      elbowJointSettings.setAlphaVelocity(0.95);
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
      integrationSettings.add(new ImmutableTriple<>("ElbowAccelerationIntegration", elbowJointSettings, elbowJointNames));

      // Wrist joints:
      JointAccelerationIntegrationParameters wristJointSettings = new JointAccelerationIntegrationParameters();
      wristJointSettings.setAlphaPosition(0.9999);
      wristJointSettings.setAlphaVelocity(0.95);
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
      integrationSettings.add(new ImmutableTriple<>("WristAccelerationIntegration", wristJointSettings, wristJointNames));

      return integrationSettings;
   }
}
