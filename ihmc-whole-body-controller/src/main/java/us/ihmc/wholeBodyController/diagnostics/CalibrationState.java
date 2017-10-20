package us.ihmc.wholeBodyController.diagnostics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;

public abstract class CalibrationState<E extends Enum<E>> extends FinishableState<E>
{
   public CalibrationState(E stateEnum)
   {
      super(stateEnum);
   }
   
   public abstract LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController();
}
