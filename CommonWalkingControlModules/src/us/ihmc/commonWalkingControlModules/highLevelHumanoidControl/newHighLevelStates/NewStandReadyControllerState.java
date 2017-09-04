package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

public class NewStandReadyControllerState extends NewHoldPositionControllerState
{
   public NewStandReadyControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepParameters standPrepSetpoints,
                                       PositionControlParameters positionControlParameters)
   {
      super(NewHighLevelControllerStates.STAND_READY, controllerToolbox, positionControlParameters);
   }
}
