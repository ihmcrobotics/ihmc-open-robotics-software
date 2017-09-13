package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandReadyControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

public class StandReadyControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandReadyControllerState standReadyControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standReadyControllerState == null)
         standReadyControllerState = new StandReadyControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                   controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                   controllerFactoryHelper.getLowLevelControllerOutput());

      return standReadyControllerState;
   }

   @Override
   public HighLevelController getStateEnum()
   {
      return HighLevelController.STAND_READY;
   }
}
