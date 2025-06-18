package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.PositionPassthroughControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class PositionPassthroughControllerStateFactory implements HighLevelControllerStateFactory
{
   private PositionPassthroughControllerState positionPassthroughControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (positionPassthroughControllerState == null)
      {
         positionPassthroughControllerState = new PositionPassthroughControllerState(controllerFactoryHelper.getCommandInputManager(),
                                                                                     controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                                     controllerFactoryHelper.getHighLevelControllerParameters());
      }

      return positionPassthroughControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.POSITION_PASSTHROUGH;
   }
}
