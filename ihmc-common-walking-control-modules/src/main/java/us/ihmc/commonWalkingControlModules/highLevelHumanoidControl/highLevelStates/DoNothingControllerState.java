package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class DoNothingControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   public DoNothingControllerState(OneDoFJoint[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters)
   {
      super(controllerState, highLevelControllerParameters, controlledJoints);
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controlledJoints.length);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void doAction(double timeInState)
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         controlledJoints[i].setTau(0.0);
         lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoints[i]).clear();
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(controlledJoints[i], 0.0);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onEntry()
   {
      // Do nothing

   }

   @Override
   public void onExit()
   {
      // Do nothing

   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
