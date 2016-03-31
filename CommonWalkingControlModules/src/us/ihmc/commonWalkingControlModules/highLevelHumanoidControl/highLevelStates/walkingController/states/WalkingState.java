package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.State;

public abstract class WalkingState extends State<WalkingStateEnum>
{
   public WalkingState(WalkingStateEnum stateEnum)
   {
      super(stateEnum);
   }

   public boolean isDoubleSupportState()
   {
      return getStateEnum().isDoubleSupport();
   }

   public boolean isSingleSupportState()
   {
      return getStateEnum().isSingleSupport();
   }

   public RobotSide getSupportSide()
   {
      return getStateEnum().getSupportSide();
   }
}
