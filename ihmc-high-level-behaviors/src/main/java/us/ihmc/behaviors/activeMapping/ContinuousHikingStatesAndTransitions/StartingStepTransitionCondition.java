package us.ihmc.behaviors.activeMapping.ContinuousHikingStatesAndTransitions;

import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class StartingStepTransitionCondition implements StateTransitionCondition
{
   private final ContinuousPlanner continuousPlanner;
   private final StepValidityChecker stepValidityChecker;
   private final ContinuousHikingParameters continuousHikingParameters;

   public StartingStepTransitionCondition(ContinuousPlanner continuousPlanner, StepValidityChecker stepValidityChecker,
                                          ContinuousHikingParameters continuousHikingParameters)
   {
      this.continuousPlanner = continuousPlanner;
      this.stepValidityChecker = stepValidityChecker;
      this.continuousHikingParameters = continuousHikingParameters;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      if (!continuousPlanner.isPlanAvailable())
         return false;
      if (!continuousHikingParameters.getStepPublisherEnabled())
         return false;

      return stepValidityChecker.isNextStepValid();
   }
}
