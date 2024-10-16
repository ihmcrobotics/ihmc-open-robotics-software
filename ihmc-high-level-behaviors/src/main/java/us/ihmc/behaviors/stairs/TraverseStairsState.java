package us.ihmc.behaviors.stairs;

import us.ihmc.commons.stateMachine.core.State;

public abstract class TraverseStairsState implements State
{
   TraverseStairsBehavior.TraverseStairsStateName previousStateName;
   public void setPreviousStateName(TraverseStairsBehavior.TraverseStairsStateName previousStateName)
   {
      this.previousStateName = previousStateName;
   }

   public TraverseStairsBehavior.TraverseStairsStateName getPreviousStateName()
   {
      return previousStateName;
   }
}
