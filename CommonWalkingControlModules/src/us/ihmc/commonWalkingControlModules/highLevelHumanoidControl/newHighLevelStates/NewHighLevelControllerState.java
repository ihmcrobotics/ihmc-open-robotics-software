package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class NewHighLevelControllerState extends FinishableState<NewHighLevelControllerStates>
{

   public NewHighLevelControllerState(NewHighLevelControllerStates stateEnum)
   {
      super(stateEnum);
   }

   @Override
   public abstract void doAction();

   @Override
   public abstract void doTransitionIntoAction();

   @Override
   public abstract void doTransitionOutOfAction();

   public abstract YoVariableRegistry getYoVariableRegistry();

   public abstract void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput);

   public abstract LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController();

   @Override
   public boolean isDone()
   {
      return false;
   }
}
