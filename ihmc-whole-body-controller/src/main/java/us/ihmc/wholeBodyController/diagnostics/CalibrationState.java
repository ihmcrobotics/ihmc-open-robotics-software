package us.ihmc.wholeBodyController.diagnostics;

import us.ihmc.commons.stateMachine.core.State;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListReadOnly;

public interface CalibrationState extends State
{
   public abstract JointDesiredOutputListReadOnly getOutputForLowLevelController();
}
