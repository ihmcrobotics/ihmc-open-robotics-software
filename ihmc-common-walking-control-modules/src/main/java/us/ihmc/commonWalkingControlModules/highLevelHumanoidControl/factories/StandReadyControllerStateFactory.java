package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandReadyControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

public class StandReadyControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandReadyControllerState standReadyControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standReadyControllerState == null)
         standReadyControllerState = new StandReadyControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                   controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                   controllerFactoryHelper.getLowLevelControllerOutput());

      return standReadyControllerState;
   }

   @Override
   public NewHighLevelControllerStates getStateEnum()
   {
      return NewHighLevelControllerStates.STAND_READY;
   }
}
