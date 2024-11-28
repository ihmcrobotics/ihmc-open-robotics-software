package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelMPCControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.MPCHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.MPCWalkingControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class MPCWalkingControllerStateFactory implements MPCHighLevelControllerStateFactory
{
   private MPCWalkingControllerState walkingControllerState;

   @Override
   public MPCHighLevelControllerState getOrCreateControllerState(HighLevelMPCControllerFactoryHelper controllerFactoryHelper)
   {
      if (walkingControllerState == null)
      {
         walkingControllerState = new MPCWalkingControllerState(controllerFactoryHelper.getCommandInputManager(), controllerFactoryHelper.getStatusMessageOutputManager(),
                                                                controllerFactoryHelper.getManagerFactory(),
                                                                controllerFactoryHelper.getWholeBodyControllerCoreFactory(),
                                                                controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                controllerFactoryHelper.getWalkingControllerParameters(),
                                                                controllerFactoryHelper.getControllerCoreOutputDataHolder());
      }

      return walkingControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.WALKING;
   }
}
