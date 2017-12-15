package us.ihmc.wanderer.parameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public class WandererHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final WandererJointMap jointMap;

   public WandererHighLevelControllerParameters(WandererJointMap jointMap)
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
      JointDesiredBehavior allJointBehaviors = new JointDesiredBehavior(JointDesiredControlMode.EFFORT);

      List<String> allJoints = new ArrayList<>(Arrays.asList(jointMap.getOrderedJointNames()));
      // for some reason these joints are not part of jointMap.getOrderedJointNames() but are controlled.
      allJoints.add("l_arm_shy");
      allJoints.add("r_arm_shy");
      allJoints.add("l_arm_shx");
      allJoints.add("r_arm_shx");
      allJoints.add("l_arm_shz");
      allJoints.add("r_arm_shz");
      allJoints.add("l_arm_ely");
      allJoints.add("r_arm_ely");
      allJoints.add("l_arm_wrz");
      allJoints.add("r_arm_wrz");

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
