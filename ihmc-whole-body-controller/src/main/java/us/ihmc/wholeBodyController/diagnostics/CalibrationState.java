package us.ihmc.wholeBodyController.diagnostics;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public interface CalibrationState extends State
{
   public abstract JointDesiredOutputListReadOnly getOutputForLowLevelController();

   public abstract WholeBodyControllerCore getWholeBodyControllerCore();
}
