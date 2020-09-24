package us.ihmc.valkyrie.simulation;

import static us.ihmc.robotics.partNames.ArmJointName.ELBOW_PITCH;
import static us.ihmc.robotics.partNames.ArmJointName.ELBOW_ROLL;
import static us.ihmc.robotics.partNames.ArmJointName.FIRST_WRIST_PITCH;
import static us.ihmc.robotics.partNames.ArmJointName.SHOULDER_PITCH;
import static us.ihmc.robotics.partNames.ArmJointName.SHOULDER_ROLL;
import static us.ihmc.robotics.partNames.ArmJointName.SHOULDER_YAW;
import static us.ihmc.robotics.partNames.ArmJointName.WRIST_ROLL;
import static us.ihmc.robotics.partNames.LegJointName.ANKLE_PITCH;
import static us.ihmc.robotics.partNames.LegJointName.ANKLE_ROLL;
import static us.ihmc.robotics.partNames.LegJointName.HIP_PITCH;
import static us.ihmc.robotics.partNames.LegJointName.HIP_ROLL;
import static us.ihmc.robotics.partNames.LegJointName.HIP_YAW;
import static us.ihmc.robotics.partNames.LegJointName.KNEE_PITCH;
import static us.ihmc.robotics.partNames.NeckJointName.DISTAL_NECK_PITCH;
import static us.ihmc.robotics.partNames.NeckJointName.DISTAL_NECK_YAW;
import static us.ihmc.robotics.partNames.NeckJointName.PROXIMAL_NECK_PITCH;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_PITCH;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_ROLL;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_YAW;
import static us.ihmc.sensorProcessing.outputData.JointDesiredControlMode.POSITION;
import static us.ihmc.valkyrie.ValkyrieHighLevelControllerParameters.configureBehavior;
import static us.ihmc.valkyrie.ValkyrieHighLevelControllerParameters.configureSymmetricBehavior;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieSimulationPositionControlParameters implements HighLevelControllerParameters
{
   private final HighLevelControllerParameters originalParameters;
   private final ValkyrieJointMap jointMap;
   private final HighLevelControllerName positionControlState;

   public ValkyrieSimulationPositionControlParameters(HighLevelControllerParameters originalParameters, ValkyrieJointMap jointMap,
                                                      HighLevelControllerName positionControlState)
   {
      this.originalParameters = originalParameters;
      this.jointMap = jointMap;
      this.positionControlState = positionControlState;
   }

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return originalParameters.getStandPrepParameters();
   }

   @Override
   public HighLevelControllerName getDefaultInitialControllerState()
   {
      return originalParameters.getDefaultInitialControllerState();
   }

   @Override
   public HighLevelControllerName getFallbackControllerState()
   {
      return originalParameters.getDefaultInitialControllerState();
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      return originalParameters.automaticallyTransitionToWalkingWhenReady();
   }

   @Override
   public double getTimeToMoveInStandPrep()
   {
      return originalParameters.getTimeToMoveInStandPrep();
   }

   @Override
   public double getMinimumTimeInStandReady()
   {
      return originalParameters.getMinimumTimeInStandReady();
   }

   @Override
   public double getTimeInStandTransition()
   {
      return originalParameters.getTimeInStandTransition();
   }

   @Override
   public double getCalibrationDuration()
   {
      return originalParameters.getCalibrationDuration();
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
   {
      if (state == positionControlState)
         return getPositionControlBehavior();
      else
         return originalParameters.getDesiredJointBehaviors(state);
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorsUnderLoad(HighLevelControllerName state)
   {
      if (state == positionControlState)
         return getPositionControlBehavior();
      else
         return originalParameters.getDesiredJointBehaviorsUnderLoad(state);
   }

   @Override
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParameters(HighLevelControllerName state)
   {
      try
      {
         return originalParameters.getJointAccelerationIntegrationParameters(state);
      }
      catch (RuntimeException e)
      {
         return null;
      }
   }

   @Override
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersUnderLoad(HighLevelControllerName state)
   {
      return originalParameters.getJointAccelerationIntegrationParametersUnderLoad(state);
   }

   @Override
   public boolean deactivateAccelerationIntegrationInTheWBC()
   {
      return originalParameters.deactivateAccelerationIntegrationInTheWBC();
   }

   protected List<GroupParameter<JointDesiredBehaviorReadOnly>> getPositionControlBehavior()
   {
      double maxPositionError = 0.05;
      double maxVelocityError = 0.5;

      List<GroupParameter<JointDesiredBehaviorReadOnly>> jointBehaviors = new ArrayList<>();

      // @formatter:off
      configureSymmetricBehavior(jointBehaviors, jointMap, SHOULDER_PITCH, POSITION     ,  8000.0, 100.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, SHOULDER_ROLL, POSITION      ,  8000.0, 100.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, SHOULDER_YAW, POSITION       ,  8000.0,  50.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, ELBOW_PITCH, POSITION        ,  8000.0,  50.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, ELBOW_ROLL, POSITION         ,  1500.0,  15.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, WRIST_ROLL, POSITION         ,   150.0,   1.5, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, FIRST_WRIST_PITCH, POSITION  ,   150.0,   1.5, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, HIP_YAW, POSITION            ,  8000.0, 100.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, HIP_ROLL, POSITION           , 10000.0, 150.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, HIP_PITCH, POSITION          , 10000.0, 150.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, KNEE_PITCH, POSITION         ,  8000.0, 100.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, ANKLE_PITCH, POSITION        ,  4000.0,  40.0, maxPositionError, maxVelocityError);
      configureSymmetricBehavior(jointBehaviors, jointMap, ANKLE_ROLL, POSITION         ,  2000.0,  30.0, maxPositionError, maxVelocityError);
      configureBehavior         (jointBehaviors, jointMap, SPINE_YAW, POSITION          , 10000.0, 250.0, maxPositionError, maxVelocityError);
      configureBehavior         (jointBehaviors, jointMap, SPINE_PITCH, POSITION        , 30000.0, 800.0, maxPositionError, maxVelocityError);
      configureBehavior         (jointBehaviors, jointMap, SPINE_ROLL, POSITION         , 30000.0, 800.0, maxPositionError, maxVelocityError);
      configureBehavior         (jointBehaviors, jointMap, PROXIMAL_NECK_PITCH, POSITION,  1500.0,  50.0, maxPositionError, maxVelocityError);
      configureBehavior         (jointBehaviors, jointMap, DISTAL_NECK_YAW, POSITION    ,  1500.0,  50.0, maxPositionError, maxVelocityError);
      configureBehavior         (jointBehaviors, jointMap, DISTAL_NECK_PITCH, POSITION  ,  1500.0,  50.0, maxPositionError, maxVelocityError);
      // @formatter:on
      return jointBehaviors;
   }
}