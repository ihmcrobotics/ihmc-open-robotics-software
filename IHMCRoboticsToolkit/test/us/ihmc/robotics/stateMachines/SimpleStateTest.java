package us.ihmc.robotics.stateMachines;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.SimpleState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;

public class SimpleStateTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleStateMachine()
   {
      SettableDoubleProvider timeProvider = new SettableDoubleProvider();
      YoVariableRegistry registry = new YoVariableRegistry("SimpleStateMachine");

      StateMachine<StateList> stateMachine = new StateMachine<StateList>("SimpleStateMachine", "switchTime", StateList.class, timeProvider, registry);

      SimpleState<StateList> stateStart = new SimpleState<StateList>(StateList.START, StateList.ONE)
      {
         @Override
         public void doAction()
         {
            if (getTimeInCurrentState() > 1.0)
               this.transitionToDefaultNextState();
         }
      };

      stateMachine.addState(stateStart);

      SimpleState<StateList> stateOne = new SimpleState<StateList>(StateList.ONE, StateList.TWO)
      {
         @Override
         public void doAction()
         {
            if (getTimeInCurrentState() > 2.0)
               this.transitionToDefaultNextState();
         }
      };

      stateMachine.addState(stateOne);

      SimpleState<StateList> stateTwo = new SimpleState<StateList>(StateList.TWO, StateList.END)
      {
         @Override
         public void doAction()
         {
            if (getTimeInCurrentState() > 3.0)
               this.transitionToDefaultNextState();
         }
      };

      stateMachine.addState(stateTwo);

      SimpleState<StateList> stateEnd = new SimpleState<StateList>(StateList.END)
      {
         @Override
         public void doAction()
         {

         }
      };

      stateMachine.addState(stateEnd);

      assertEquals(stateStart, stateMachine.getCurrentState());

      stateMachine.setCurrentState(StateList.START);
      assertEquals(stateMachine.getCurrentState(), stateStart);

      while (timeProvider.getValue() < 10.0)
      {
         stateMachine.doAction();
         stateMachine.checkTransitionConditions();
         timeProvider.add(0.01);
      }

      assertEquals(stateMachine.getCurrentState(), stateEnd);
   }

   private enum StateList
   {
      START, ONE, TWO, END;
   }

}
