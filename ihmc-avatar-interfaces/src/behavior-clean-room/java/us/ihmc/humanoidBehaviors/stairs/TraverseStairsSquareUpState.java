package us.ihmc.humanoidBehaviors.stairs;

import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;

public class TraverseStairsSquareUpState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;

   public TraverseStairsSquareUpState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters)
   {
      this.helper = helper;
      this.parameters = parameters;
   }

   @Override
   public void onEntry()
   {
      LogTools.debug("Entering " + getClass().getSimpleName());
   }

   @Override
   public void doAction(double timeInState)
   {

   }

   @Override
   public boolean isDone(double timeInState)
   {
      LogTools.debug(getClass().getSimpleName() + " is done");
      return true;
   }
}
