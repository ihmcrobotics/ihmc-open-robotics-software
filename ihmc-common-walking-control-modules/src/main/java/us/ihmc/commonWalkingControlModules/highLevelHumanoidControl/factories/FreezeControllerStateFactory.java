package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.FreezeControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

public class FreezeControllerStateFactory implements HighLevelControllerStateFactory
{
   private FreezeControllerState freezeControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (freezeControllerState == null)
         freezeControllerState = new FreezeControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                           controllerFactoryHelper.getHighLevelControllerParameters());

      return freezeControllerState;
   }

   @Override
   public HighLevelController getStateEnum()
   {
      return HighLevelController.FREEZE_STATE;
   }
}
