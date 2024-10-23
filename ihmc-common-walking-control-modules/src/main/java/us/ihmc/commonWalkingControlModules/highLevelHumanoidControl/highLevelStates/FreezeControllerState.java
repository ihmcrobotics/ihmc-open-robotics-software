package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListReadOnly;

public class FreezeControllerState extends HoldPositionControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.FREEZE_STATE;

   public FreezeControllerState(OneDoFJointBasics[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters,
                                JointDesiredOutputListReadOnly highLevelControllerOutput)
   {
      super(controllerState, controlledJoints, highLevelControllerParameters, highLevelControllerOutput);
   }
}
