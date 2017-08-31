package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

public class NewFreezeControllerState extends NewHoldPositionControllerState
{
   public NewFreezeControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepParameters standPrepSetpoints,
                                   PositionControlParameters positionControlParameters)
   {
      super(NewHighLevelControllerStates.FREEZE_STATE, controllerToolbox, standPrepSetpoints, positionControlParameters);
   }
}
