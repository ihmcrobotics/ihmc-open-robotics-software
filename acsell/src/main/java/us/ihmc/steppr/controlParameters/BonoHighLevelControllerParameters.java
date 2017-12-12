package us.ihmc.steppr.controlParameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointGroupParameter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.steppr.parameters.BonoJointMap;

public class BonoHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final BonoJointMap jointMap;

   public BonoHighLevelControllerParameters(BonoJointMap jointMap)
   {
      this.jointMap = jointMap;
   }

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return null;
   }

   @Override
   public List<JointGroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
   {
      JointDesiredBehavior allJointBehaviors = new JointDesiredBehavior(JointDesiredControlMode.EFFORT);

      List<String> allJoints = Arrays.asList(jointMap.getOrderedJointNames());
      List<JointGroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();
      behaviors.add(new JointGroupParameter<>("", allJointBehaviors, allJoints));
      return behaviors;
   }

   @Override
   public HighLevelControllerName getDefaultInitialControllerState()
   {
      return HighLevelControllerName.DO_NOTHING_BEHAVIOR;
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
}
