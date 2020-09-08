package us.ihmc.humanoidBehaviors.stairs;

import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class TraverseStairsPlanStepsState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;

   public TraverseStairsPlanStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters)
   {
      this.helper = helper;
      this.parameters = parameters;
   }

   @Override
   public void onEntry()
   {

   }

   @Override
   public void doAction(double timeInState)
   {

   }

   boolean shouldTransitionToExecute(double timeInState)
   {
      return false;
   }

   boolean shouldTransitionBackToPause(double timeInState)
   {
      return false;
   }
}
