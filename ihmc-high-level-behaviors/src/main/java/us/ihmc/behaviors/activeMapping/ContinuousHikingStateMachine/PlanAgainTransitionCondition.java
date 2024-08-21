package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class PlanAgainTransitionCondition implements StateTransitionCondition
{
   private final ContinuousPlanner continuousPlanner;
   private final ContinuousHikingParameters continuousHikingParameters;

   /**
    * This transition is used in the {@link us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask} to determine whether the Continuous Hiking state
    * machine should plan again or not. We allow planning over and over to see if things need to be tuned. This means we can plan without walking, tune some
    * parameters, and plan again to see how things have changed.
    */
   public PlanAgainTransitionCondition(ContinuousPlanner continuousPlanner, ContinuousHikingParameters continuousHikingParameters)
   {
      this.continuousPlanner = continuousPlanner;
      this.continuousHikingParameters = continuousHikingParameters;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      return continuousPlanner.isPlanAvailable() && !continuousHikingParameters.getStepPublisherEnabled();
   }
}
