package us.ihmc.wholeBodyController.diagnostics;

import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.outputData.JointDesiredOutputListReadOnly;

public interface CalibrationState extends State
{
   public abstract JointDesiredOutputListReadOnly getOutputForLowLevelController();
}
