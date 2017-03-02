package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;

public class BehaviorStateMachine<E extends Enum<E>> extends GenericStateMachine<E, BehaviorAction<E>>
{

   protected E startState;

   public BehaviorStateMachine(String name, String switchTimeName, Class<E> enumType, DoubleYoVariable t, YoVariableRegistry registry)
   {
      super(name, switchTimeName, enumType, t, registry);
   }

   public void initialize()
   {
      try
      {
         if (startState == null)
            throw new Exception(
                  "Start State Not Set for" + getStateYoVariableName() + " use statemachine.setStartState(Enume state) when setting up the state machine");
         setCurrentState(startState);

      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   public void setStartState(E state)
   {
      this.startState = state;
   }

   public void addStateWithDoneTransition(BehaviorAction<E> state, E nextState)
   {
      state.addDoneWithStateTransition(nextState);
      addState(state);
   }

   public void pause()
   {
      BehaviorAction<E> currentState = getCurrentState();
      currentState.pause();
   }

   public void resume()
   {
      BehaviorAction<E> currentState = getCurrentState();
      currentState.resume();
   }

   public void stop()
   {
      BehaviorAction<E> currentState = getCurrentState();
      currentState.abort();
   }

   public void doPostBehaviorCleanup()
   {
      BehaviorAction<E> currentState = getCurrentState();
      currentState.doPostBehaviorCleanup();
   }

}
