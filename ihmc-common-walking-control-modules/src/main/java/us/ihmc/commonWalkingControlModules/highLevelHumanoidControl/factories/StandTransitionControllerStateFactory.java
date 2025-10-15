package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class StandTransitionControllerStateFactory extends SmoothTransitionControllerStateFactory
{
   public StandTransitionControllerStateFactory()
   {
      this(HighLevelControllerName.STAND_READY, HighLevelControllerName.WALKING);
   }

   public StandTransitionControllerStateFactory(HighLevelControllerName startState, HighLevelControllerName endState)
   {
      super("toWalking", HighLevelControllerName.STAND_TRANSITION_STATE, startState, endState);
   }
}
