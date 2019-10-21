package us.ihmc.humanoidBehaviors.stateMachine;

import java.util.Map;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;

public class BehaviorStateMachine<E extends Enum<E>>
{
   private final StateMachine<E, BehaviorAction> stateMachine;

   public BehaviorStateMachine(StateMachine<E, BehaviorAction> stateMachine)
   {
      this.stateMachine = stateMachine;
   }

   public void initialize()
   {
      stateMachine.resetToInitialState();
   }

   public void resetCurrentState()
   {
      stateMachine.resetCurrentState();
   }

   public void doControlAndTransitions()
   {
      stateMachine.doActionAndTransition();
   }

   public void pause()
   {
      BehaviorAction currentState = stateMachine.getCurrentState();
      currentState.pause();
   }

   public void resume()
   {
      BehaviorAction currentState = stateMachine.getCurrentState();
      currentState.resume();
   }

   public void stop()
   {
      BehaviorAction currentState = stateMachine.getCurrentState();
      currentState.abort();
   }

   public void doPostBehaviorCleanup()
   {
      BehaviorAction currentState = stateMachine.getCurrentState();
      currentState.doPostBehaviorCleanup();
   }

   public BehaviorAction getCurrentAction()
   {
      return stateMachine.getCurrentState();
   }

   public E getCurrentBehaviorKey()
   {
      return stateMachine.getCurrentStateKey();
   }
   
   

   public boolean isCurrentActionTerminal()
   {
      return stateMachine.isCurrentStateTerminal();
   }
   
   public double getTimeInCurrentState()
   {
      return stateMachine.getTimeInCurrentState();
   }
}
