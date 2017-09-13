package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

public interface HighLevelControllerStateFactory
{
   HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper);

   HighLevelController getStateEnum();

   default boolean isTransitionToControllerRequested()
   {
      return false;
   }
}
