package us.ihmc.robotics.stateMachine;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StateMachineExampleOneTest
{
   @Test
   public void testConstructionOfSimpleTimeBasedStateMachine()
   {
      // This shows a quick and easy way to make a simple state machine without using the complex machinery for a more complex state machine.
      // This one goes through states Start, StateOne, StateTwo, and Stop.
      // It transitions based on time.
      YoRegistry registry = new YoRegistry("registry");
      YoDouble time = new YoDouble("time", registry);
      StateMachineFactory<ExampleStateName, State> factory = new StateMachineFactory<>(ExampleStateName.class);
      factory.setNamePrefix("blah");
      factory.setRegistry(registry);
      factory.buildClock(time);

      SimpleExampleState startingState = new SimpleExampleState(ExampleStateName.Starting);
      SimpleExampleState stateOne = new SimpleExampleState(ExampleStateName.StateOne);
      SimpleExampleState stateTwo = new SimpleExampleState(ExampleStateName.StateTwo);
      SimpleExampleState stoppedState = new SimpleExampleState(ExampleStateName.Stopped);

      factory.addState(ExampleStateName.Starting, startingState);
      factory.addState(ExampleStateName.StateOne, stateOne);
      factory.addState(ExampleStateName.StateTwo, stateTwo);
      factory.addState(ExampleStateName.Stopped, stoppedState);

      factory.addTimeBasedTransition(ExampleStateName.Starting, ExampleStateName.StateOne, 1.0);
      factory.addTimeBasedTransition(ExampleStateName.StateOne, ExampleStateName.StateTwo, 1.0);
      factory.addTimeBasedTransition(ExampleStateName.StateTwo, ExampleStateName.Stopped, 1.0);

      StateMachine<ExampleStateName, State> stateMachine = factory.build(ExampleStateName.Starting);
      stateMachine.resetToInitialState();

      // Some simple tests. Just tick through and make sure we are in the correct state at the correct time:
      do
      {
         ExampleStateName currentStateEnum = stateMachine.getCurrentStateKey();
         if (time.getValue() < 1.01)
         {
            assertEquals(ExampleStateName.Starting, currentStateEnum);
            assertFalse(stateMachine.getTimeInCurrentState() >= 1.01);
         }
         else if (time.getValue() < 2.01)
         {
            assertEquals(ExampleStateName.StateOne, currentStateEnum);
         }
         else if (time.getValue() < 3.01)
            assertEquals(ExampleStateName.StateTwo, currentStateEnum);
         else
            assertEquals(ExampleStateName.Stopped, currentStateEnum);

         SimpleExampleState currentState = (SimpleExampleState) stateMachine.getCurrentState();

         if (stateMachine.getTimeInCurrentState() < 0.001)
         {
            assertTrue(currentState.didTransitionIntoAction);
            assertFalse(currentState.didAction);
            assertFalse(currentState.didTransitionOutOfAction);
         }

         boolean wontTransition = stateMachine.getTimeInCurrentState() < 1.0;

         stateMachine.doActionAndTransition();
         time.set(time.getValue() + 0.01);

         double timeInCurrentState = stateMachine.getTimeInCurrentState();
         assertTrue(stateMachine.getTimeInCurrentState() >= timeInCurrentState);
         assertFalse(stateMachine.getTimeInCurrentState() >= (timeInCurrentState + 1e-5));

         assertTrue(currentState.didTransitionIntoAction);
         assertTrue(currentState.didAction);

         if (wontTransition)
         {
            assertFalse(currentState.didTransitionOutOfAction);
         }
         else if (currentState != stoppedState)
         {
            assertTrue(currentState.didTransitionOutOfAction);
         }

         assertEquals(currentState.getStateEnum(), currentStateEnum);
      }
      while (time.getValue() < 10.0);

   }

   @Test
   public void testConstructionOfSimpleEventBasedStateMachine()
   {
      // This shows a quick and easy way to make a simple state machine without using the complex machinery for a more complex state machine.
      // This one goes through states Start, StateOne, StateTwo, and Stop.
      // It transitions based on calling the transitionToDefaultNextState() method in the State...

      YoRegistry registry = new YoRegistry("registry");
      YoDouble time = new YoDouble("time", registry);
      StateMachineFactory<ExampleStateName, SimpleExampleState> factory = new StateMachineFactory<>(ExampleStateName.class);
      factory.setNamePrefix("bop");
      factory.setRegistry(registry);

      SimpleExampleState startingState = new SimpleExampleState(ExampleStateName.Starting);
      SimpleExampleState stateOne = new SimpleExampleState(ExampleStateName.StateOne);
      SimpleExampleState stateTwo = new SimpleExampleState(ExampleStateName.StateTwo);
      SimpleExampleState stoppedState = new SimpleExampleState(ExampleStateName.Stopped);

      factory.addStateAndDoneTransition(ExampleStateName.Starting, startingState, ExampleStateName.StateOne);
      factory.addStateAndDoneTransition(ExampleStateName.StateOne, stateOne, ExampleStateName.StateTwo);
      factory.addStateAndDoneTransition(ExampleStateName.StateTwo, stateTwo, ExampleStateName.Stopped);
      factory.addState(ExampleStateName.Stopped, stoppedState);

      StateMachine<ExampleStateName, SimpleExampleState> stateMachine = factory.build(ExampleStateName.Starting);

      // Some simple tests. Just tick through and make sure we transition through the states properly:

      assertEquals(startingState, stateMachine.getState(ExampleStateName.Starting));
      assertEquals(stateOne, stateMachine.getState(ExampleStateName.StateOne));
      assertEquals(stateTwo, stateMachine.getState(ExampleStateName.StateTwo));
      assertEquals(stoppedState, stateMachine.getState(ExampleStateName.Stopped));

      stateMachine.resetToInitialState();
      assertEquals(ExampleStateName.Starting, stateMachine.getCurrentStateKey());

      assertEquals(startingState, stateMachine.getState(ExampleStateName.Starting));
      assertEquals(stateOne, stateMachine.getState(ExampleStateName.StateOne));
      assertEquals(stateTwo, stateMachine.getState(ExampleStateName.StateTwo));
      assertEquals(stoppedState, stateMachine.getState(ExampleStateName.Stopped));

      RememberStateChangedListener listener = new RememberStateChangedListener();
      stateMachine.addStateChangedListener(listener);

      RememberStateChangedListener listenerTwo = new RememberStateChangedListener();
      stateMachine.addStateChangedListener(listenerTwo);

      ExampleStateName currentStateEnum = stateMachine.getCurrentStateKey();
      SimpleExampleState currentState = (SimpleExampleState) stateMachine.getCurrentState();

      assertEquals(currentState, startingState);
      assertEquals(currentStateEnum, ExampleStateName.Starting);

      // Not sure if this should be Starting or null here. Hmm...
      //      assertEquals(currentState.getPreviousState(), startingState); TODO fixme?

      assertTrue(currentState.didTransitionIntoAction);
      assertFalse(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      stateMachine.doActionAndTransition();
      time.set(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      assertNull(listener.oldState);
      assertNull(listener.newState);

      assertEquals(ExampleStateName.Starting, stateMachine.getCurrentStateKey());
      currentState.isDone = true;

      stateMachine.doActionAndTransition();

      assertEquals(ExampleStateName.StateOne, stateMachine.getCurrentStateKey());

      assertEquals(startingState.getStateEnum(), listener.oldState);
      assertEquals(stateOne.getStateEnum(), listener.newState);

      time.set(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateKey();
      currentState = (SimpleExampleState) stateMachine.getCurrentState();

      assertEquals(currentState, stateOne);
      assertEquals(ExampleStateName.StateOne, currentStateEnum);
      //      assertEquals(currentState.getPreviousState(), startingState); TODO fixme?

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      assertEquals(startingState.getStateEnum(), listener.oldState);
      assertEquals(stateOne.getStateEnum(), listener.newState);

      assertEquals(ExampleStateName.StateOne, stateMachine.getCurrentStateKey());

      currentState.isDone = true;

      stateMachine.doActionAndTransition();

      assertEquals(ExampleStateName.StateTwo, stateMachine.getCurrentStateKey());

      assertEquals(stateOne.getStateEnum(), listener.oldState);
      assertEquals(stateTwo.getStateEnum(), listener.newState);
      assertEquals(stateOne.getStateEnum(), listenerTwo.oldState);
      assertEquals(stateTwo.getStateEnum(), listenerTwo.newState);

      time.set(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateKey();
      currentState = (SimpleExampleState) stateMachine.getCurrentState();

      assertEquals(ExampleStateName.StateTwo, stateMachine.getCurrentStateKey());
      assertEquals(currentState, stateTwo);
      assertEquals(currentStateEnum, ExampleStateName.StateTwo);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      stateMachine.doActionAndTransition();
      time.set(time.getValue() + 0.01);

      assertEquals(stateTwo, stateMachine.getCurrentState());
      assertEquals(ExampleStateName.StateTwo, stateMachine.getCurrentStateKey());

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      assertEquals(stateOne.getStateEnum(), listener.oldState);
      assertEquals(stateTwo.getStateEnum(), listener.newState);

      assertEquals(ExampleStateName.StateTwo, stateMachine.getCurrentStateKey());

      currentState.isDone = true;

      stateMachine.doActionAndTransition();

      assertEquals(ExampleStateName.Stopped, stateMachine.getCurrentStateKey());

      assertEquals(stateTwo.getStateEnum(), listener.oldState);
      assertEquals(stoppedState.getStateEnum(), listener.newState);

      time.set(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateKey();
      currentState = (SimpleExampleState) stateMachine.getCurrentState();

      assertEquals(currentState, stoppedState);
      assertEquals(currentStateEnum, ExampleStateName.Stopped);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      stateMachine.doActionAndTransition();
      time.set(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);
   }

   private class SimpleExampleState implements State
   {
      public boolean didTransitionIntoAction = false;
      public boolean didAction = false;
      public boolean didTransitionOutOfAction = false;
      private final ExampleStateName stateEnum;
      private boolean isDone = false;

      public SimpleExampleState(ExampleStateName stateEnum)
      {
         this.stateEnum = stateEnum;
      }

      @Override
      public void doAction(double timeInState)
      {
         didAction = true;
      }

      @Override
      public void onEntry()
      {
         didTransitionIntoAction = true;
      }

      @Override
      public void onExit()
      {
         didTransitionOutOfAction = true;
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone;
      }

      public ExampleStateName getStateEnum()
      {
         return stateEnum;
      }
   }

   private enum ExampleStateName
   {
      Starting, StateOne, StateTwo, Stopped
   }

   private class RememberStateChangedListener implements StateChangedListener<ExampleStateName>
   {
      public ExampleStateName oldState;
      public ExampleStateName newState;

      @Override
      public void stateChanged(ExampleStateName oldState, ExampleStateName newState)
      {
         this.oldState = oldState;
         this.newState = newState;
      };
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(StateMachineExampleOneTest.class);
   }

}
