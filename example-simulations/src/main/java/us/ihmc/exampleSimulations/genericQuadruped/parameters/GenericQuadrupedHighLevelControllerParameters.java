package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.dataStructures.parameters.GroupParameter;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.QuadrupedJointNameMap;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

import java.util.ArrayList;
import java.util.List;

public class GenericQuadrupedHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final GenericQuadrupedStandPrepParameters standPrepParameters = new GenericQuadrupedStandPrepParameters();
   private final List<GroupParameter<JointDesiredBehaviorReadOnly>> walkingJointBehavior = new ArrayList<>();
   private final List<GroupParameter<JointDesiredBehaviorReadOnly>> nonWalkingJointBehavior = new ArrayList<>();
   private final QuadrupedJointNameMap jointMap;

   public GenericQuadrupedHighLevelControllerParameters(QuadrupedJointNameMap jointMap)
   {
      this.jointMap = jointMap;
      setUpNonWalkingJointBehavior();
      setUpWalkingJointBehavior();
   }

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return standPrepParameters;
   }

   @Override
   public HighLevelControllerName getDefaultInitialControllerState()
   {
      return HighLevelControllerName.FREEZE_STATE;
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
      return 1.5;
   }

   @Override
   public double getMinimumTimeInStandReady()
   {
      return 0.25;
   }

   @Override
   public double getTimeInStandTransition()
   {
      return 1.0;
   }

   @Override
   public double getCalibrationDuration()
   {
      throw new RuntimeException("Generic quadruped doesn't have a calibration state");
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
   {
      switch(state)
      {
      case WALKING:
         return walkingJointBehavior;
      default:
         return nonWalkingJointBehavior;
      }
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorsUnderLoad(HighLevelControllerName state)
   {
      if (state == HighLevelControllerName.WALKING)
         return walkingJointBehavior;
      else
         return nonWalkingJointBehavior;
   }

   @Override
   public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParameters(HighLevelControllerName state)
   {
      switch (state)
      {
      case WALKING:
         return getJointAccelerationIntegrationParametersForWalking();
      case DO_NOTHING_BEHAVIOR:
      case STAND_PREP_STATE:
      case STAND_READY:
      case STAND_TRANSITION_STATE:
      case EXIT_WALKING:
      case FREEZE_STATE:
      case CUSTOM1:
         return getJointAccelerationIntegrationParametersForHangingAround();
      default:
         throw new RuntimeException("Implement a desired joint behavior for the high level state " + state);
      }
   }

   private void setUpWalkingJointBehavior()
   {
      configureSymmetricBehavior(walkingJointBehavior, jointMap, LegJointName.HIP_ROLL, JointDesiredControlMode.EFFORT, 0.0, 1.0, 0.1);
      configureSymmetricBehavior(walkingJointBehavior, jointMap, LegJointName.HIP_PITCH, JointDesiredControlMode.EFFORT, 0.0, 1.0, 0.1);
      configureSymmetricBehavior(walkingJointBehavior, jointMap, LegJointName.KNEE_PITCH, JointDesiredControlMode.EFFORT, 0.0, 1.0, 0.1);
   }

   private void setUpNonWalkingJointBehavior()
   {
      configureSymmetricBehavior(nonWalkingJointBehavior, jointMap, LegJointName.HIP_ROLL, JointDesiredControlMode.EFFORT, 500.0, 25.0, 1.0);
      configureSymmetricBehavior(nonWalkingJointBehavior, jointMap, LegJointName.HIP_PITCH, JointDesiredControlMode.EFFORT, 500.0, 25.0, 1.0);
      configureSymmetricBehavior(nonWalkingJointBehavior, jointMap, LegJointName.KNEE_PITCH, JointDesiredControlMode.EFFORT, 500.0, 25.0, 1.0);
   }

   private static void configureSymmetricBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviorParameterList, QuadrupedJointNameMap jointMap,
                                                  LegJointName jointName, JointDesiredControlMode controlMode, double stiffness, double damping, double velocityScaling)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping, 1.0, velocityScaling);
      behaviorParameterList.add(new GroupParameter<>(jointName.toString(), jointBehavior, getJointNamesForEachQuadrant(jointMap, jointName)));
   }

   private static List<String> getJointNamesForEachQuadrant(QuadrupedJointNameMap jointMap, LegJointName legJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         jointNames.add(jointMap.getLegJointName(quadrant, legJointName));
      }
      return jointNames;
   }

   private List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForWalking()
   {
      List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> ret = new ArrayList<>();

      for (LegJointName legJointName : new LegJointName[] {LegJointName.HIP_PITCH, LegJointName.HIP_ROLL, LegJointName.KNEE_PITCH})
      { // Hip joints
         JointAccelerationIntegrationParameters parameters = new JointAccelerationIntegrationParameters();
         parameters.setPositionBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.9992, 0.004));
         parameters.setVelocityBreakFrequency(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.85, 0.004));
         List<String> jointNames = new ArrayList<>();
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            jointNames.add(jointMap.getLegJointName(robotQuadrant, legJointName));
         ret.add(new GroupParameter<>(legJointName.getCamelCaseName(), parameters, jointNames));
      }

      return ret;
   }

   private List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersForHangingAround()
   {
      // Possible ass a single parameter that is shared between all joints here.
      return null;
   }
}
