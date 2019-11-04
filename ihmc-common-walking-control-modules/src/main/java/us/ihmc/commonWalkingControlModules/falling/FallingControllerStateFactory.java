package us.ihmc.commonWalkingControlModules.falling;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class FallingControllerStateFactory implements HighLevelControllerStateFactory
{
   private FallingControllerState fallingControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (fallingControllerState == null)
      {
         fallingControllerState = new FallingControllerState(controllerFactoryHelper.getCommandInputManager(),
                                                             controllerFactoryHelper.getStatusMessageOutputManager(),
                                                             controllerFactoryHelper.getManagerFactory(),
                                                             controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                             controllerFactoryHelper.getHighLevelControllerParameters(),
                                                             controllerFactoryHelper.getWalkingControllerParameters());
      }

      return fallingControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.FALLING_STATE;
   }
}
