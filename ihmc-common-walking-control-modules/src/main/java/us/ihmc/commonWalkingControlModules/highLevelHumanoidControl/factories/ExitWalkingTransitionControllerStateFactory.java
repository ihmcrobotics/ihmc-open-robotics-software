package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class ExitWalkingTransitionControllerStateFactory extends SmoothTransitionControllerStateFactory
{
   public ExitWalkingTransitionControllerStateFactory(HighLevelControllerName endState)
   {
      super("exitWalking", HighLevelControllerName.EXIT_WALKING, HighLevelControllerName.WALKING, endState);
   }
}
