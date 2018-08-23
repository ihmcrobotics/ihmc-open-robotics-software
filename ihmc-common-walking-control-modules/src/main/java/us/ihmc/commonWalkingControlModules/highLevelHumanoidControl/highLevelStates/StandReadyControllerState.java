package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class StandReadyControllerState extends HoldPositionControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.STAND_READY;

   public StandReadyControllerState(OneDoFJoint[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters,
                                    JointDesiredOutputListReadOnly highLevelControllerOutput)
   {
      super(controllerState, controlledJoints, highLevelControllerParameters, highLevelControllerOutput);
   }
}
