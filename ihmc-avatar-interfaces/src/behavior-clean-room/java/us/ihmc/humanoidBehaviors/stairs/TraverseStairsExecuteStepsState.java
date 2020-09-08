package us.ihmc.humanoidBehaviors.stairs;

import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.robotics.stateMachine.core.State;

public class TraverseStairsExecuteStepsState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;

   public TraverseStairsExecuteStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters)
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

   boolean isCompleted()
   {
      return false;
   }
}
