package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.google.common.base.CaseFormat;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class HighLevelControllerState extends FinishableState<HighLevelControllerName>
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final JointDesiredBehaviorReadOnly[] jointBehaviors;
   private final JointDesiredOutputList stateSpecificJointLevelSettings;

   public HighLevelControllerState(HighLevelControllerName stateEnum, HighLevelControllerParameters parameters,
                                   HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      super(stateEnum);

      String stateName = CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, stateEnum.toString());
      List<GroupParameter<JointDesiredBehaviorReadOnly>> desiredJointBehaviors = parameters.getDesiredJointBehaviors(stateEnum);
      Map<String, JointDesiredBehaviorReadOnly> jointBehaviorMap = new HashMap<>();
      ParameterTools.extractJointBehaviorMap(stateName, desiredJointBehaviors, jointBehaviorMap, registry);

      OneDoFJoint[] controlledJoints = ScrewTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class);
      jointBehaviors = new JointDesiredBehaviorReadOnly[controlledJoints.length];
      stateSpecificJointLevelSettings = new JointDesiredOutputList(controlledJoints);

      for (int jointIdx = 0; jointIdx < controlledJoints.length; jointIdx++)
      {
         String jointName = stateSpecificJointLevelSettings.getJointName(jointIdx);
         JointDesiredBehaviorReadOnly jointDesiredBehavior = jointBehaviorMap.get(jointName);
         if (jointDesiredBehavior == null)
         {
            String error = "In high level state " + stateEnum + "\n"
                  + "The class " + parameters.getClass().getSimpleName() + " must define a desired joint behavior for " + jointName + ".";
            throw new RuntimeException(error);
         }
         jointBehaviors[jointIdx] = jointDesiredBehavior;
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   protected JointDesiredOutputList getStateSpecificJointSettings()
   {
      // copy the settings for the joint behavior into the joint desired output
      for (int jointIdx = 0; jointIdx < stateSpecificJointLevelSettings.getNumberOfJointsWithDesiredOutput(); jointIdx++)
      {
         JointDesiredBehaviorReadOnly jointDesiredBehavior = jointBehaviors[jointIdx];
         if (jointDesiredBehavior != null)
         {
            JointDesiredOutput jointDesiredOutput = stateSpecificJointLevelSettings.getJointDesiredOutput(jointIdx);
            jointDesiredOutput.setControlMode(jointDesiredBehavior.getControlMode());
            jointDesiredOutput.setStiffness(jointDesiredBehavior.getStiffness());
            jointDesiredOutput.setDamping(jointDesiredBehavior.getDamping());
            jointDesiredOutput.setMasterGain(jointDesiredBehavior.getMasterGain());
            jointDesiredOutput.setVelocityScaling(jointDesiredBehavior.getVelocityScaling());
         }
      }
      return stateSpecificJointLevelSettings;
   }

   public abstract JointDesiredOutputListReadOnly getOutputForLowLevelController();

   @Override
   public boolean isDone()
   {
      return false;
   }
}
