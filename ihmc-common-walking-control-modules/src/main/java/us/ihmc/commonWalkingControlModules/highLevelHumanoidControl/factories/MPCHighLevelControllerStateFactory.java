package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelMPCControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.MPCHighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public interface MPCHighLevelControllerStateFactory extends HighLevelControllerStateFactory
{
   MPCHighLevelControllerState getOrCreateControllerState(HighLevelMPCControllerFactoryHelper controllerFactoryHelper);

   HighLevelControllerName getStateEnum();

   default boolean isTransitionToControllerRequested()
   {
      return false;
   }
}
