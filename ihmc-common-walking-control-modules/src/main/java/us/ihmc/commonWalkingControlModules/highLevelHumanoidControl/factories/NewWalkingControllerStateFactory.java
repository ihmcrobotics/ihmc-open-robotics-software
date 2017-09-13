package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.NewWalkingControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

public class NewWalkingControllerStateFactory implements HighLevelControllerStateFactory
{
   private NewWalkingControllerState walkingControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (walkingControllerState == null)
      {
         walkingControllerState = new NewWalkingControllerState(controllerFactoryHelper.getCommandInputManager(), controllerFactoryHelper.getStatusMessageOutputManager(),
                                                                controllerFactoryHelper.getManagerFactory(), controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                controllerFactoryHelper.getWalkingControllerParameters());
      }

      return walkingControllerState;
   }

   @Override
   public HighLevelController getStateEnum()
   {
      return HighLevelController.WALKING;
   }
}
