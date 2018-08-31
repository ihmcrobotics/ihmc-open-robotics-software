package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedJointNameMapAndContactDefinition;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
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
   private final GenericQuadrupedJointNameMapAndContactDefinition jointMap;

   public GenericQuadrupedHighLevelControllerParameters(GenericQuadrupedJointNameMapAndContactDefinition jointMap)
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
      return HighLevelControllerName.WALKING;
   }

   @Override
   public HighLevelControllerName getFallbackControllerState()
   {
      return HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      return true;
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
      case STAND_TRANSITION_STATE:
         return walkingJointBehavior;
      default:
         return nonWalkingJointBehavior;
      }
   }

   private void setUpWalkingJointBehavior()
   {
      configureSymmetricBehavior(walkingJointBehavior, jointMap, LegJointName.HIP_ROLL, JointDesiredControlMode.EFFORT, 0.0, 0.0);
      configureSymmetricBehavior(walkingJointBehavior, jointMap, LegJointName.HIP_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
      configureSymmetricBehavior(walkingJointBehavior, jointMap, LegJointName.KNEE_PITCH, JointDesiredControlMode.EFFORT, 0.0, 0.0);
   }

   private void setUpNonWalkingJointBehavior()
   {
      configureSymmetricBehavior(nonWalkingJointBehavior, jointMap, LegJointName.HIP_ROLL, JointDesiredControlMode.POSITION, 500.0, 10.0);
      configureSymmetricBehavior(nonWalkingJointBehavior, jointMap, LegJointName.HIP_PITCH, JointDesiredControlMode.POSITION, 500.0, 10.0);
      configureSymmetricBehavior(nonWalkingJointBehavior, jointMap, LegJointName.KNEE_PITCH, JointDesiredControlMode.POSITION, 500.0, 10.0);
   }

   private static void configureSymmetricBehavior(List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviorParameterList, QuadrupedJointNameMap jointMap,
                                                  LegJointName jointName, JointDesiredControlMode controlMode, double stiffness, double damping)
   {
      JointDesiredBehavior jointBehavior = new JointDesiredBehavior(controlMode, stiffness, damping);
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
}
