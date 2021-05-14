package us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.SimpleState;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;

public class SimpleStateTest
{
   @Test
   public void testSimpleStateMachine()
   {
      SettableDoubleProvider timeProvider = new SettableDoubleProvider();
      YoRegistry registry = new YoRegistry("SimpleStateMachine");

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
