package us.ihmc.thor.parameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public class ThorHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final ThorJointMap jointMap;

   public ThorHighLevelControllerParameters(ThorJointMap jointMap)
   {
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
      JointDesiredBehavior allJointBehaviors = new JointDesiredBehavior(JointDesiredControlMode.EFFORT, 100.0, 0.5);

      List<String> allJoints = Arrays.asList(jointMap.getOrderedJointNames());
      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();
      behaviors.add(new GroupParameter<>("", allJointBehaviors, allJoints));
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
