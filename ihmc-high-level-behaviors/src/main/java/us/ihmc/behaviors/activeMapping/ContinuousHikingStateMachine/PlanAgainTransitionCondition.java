package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class PlanAgainTransitionCondition implements StateTransitionCondition
{
   private final ContinuousPlanner continuousPlanner;
   private final ContinuousHikingParameters continuousHikingParameters;

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
