package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewWalkingControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

public class NewWalkingControllerStateFactory implements HighLevelControllerStateFactory
{
   private NewWalkingControllerState walkingControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (walkingControllerState == null)
      {
         walkingControllerState = new NewWalkingControllerState(controllerFactoryHelper.getCommandInputManager(), controllerFactoryHelper.getStatusMessageOutputManager(),
                                                                controllerFactoryHelper.getManagerFactory(), controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                controllerFactoryHelper.getHighLevelControllerParameters(), controllerFactoryHelper.getIcpPlannerParameters(),
                                                                controllerFactoryHelper.getWalkingControllerParameters());
      }

      return walkingControllerState;
   }

   @Override
   public NewHighLevelControllerStates getStateEnum()
   {
      return NewHighLevelControllerStates.WALKING_STATE;
   }
}
