package us.ihmc.robotics.stateMachines;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateChangedListener;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;

public class StateMachineExampleOneTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructionOfSimpleTimeBasedStateMachine()
   {
      // This shows a quick and easy way to make a simple state machine without using the complex machinery for a more complex state machine.
      // This one goes through states Start, StateOne, StateTwo, and Stop.
      // It transitions based on time.
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      SettableDoubleProvider time = new SettableDoubleProvider();
      StateMachine<ExampleStateName> stateMachine = new StateMachine<ExampleStateName>("stateMachine", "switchTime", ExampleStateName.class, time, registry);

      SimpleExampleState startingState = new SimpleExampleState(ExampleStateName.Starting);
      startingState.addDelayBasedStateTransition(ExampleStateName.StateOne, 1.0);
      stateMachine.addState(startingState);

      SimpleExampleState stateOne = new SimpleExampleState(ExampleStateName.StateOne);
      stateOne.addDelayBasedStateTransition(ExampleStateName.StateTwo, 1.0);
      stateMachine.addState(stateOne);

      SimpleExampleState stateTwo = new SimpleExampleState(ExampleStateName.StateTwo);
      stateTwo.addDelayBasedStateTransition(ExampleStateName.Stopped, 1.0);
      stateMachine.addState(stateTwo);

      SimpleExampleState stoppedState = new SimpleExampleState(ExampleStateName.Stopped);
      stateMachine.addState(stoppedState);

      stateMachine.setCurrentState(ExampleStateName.Starting);

      // Some simple tests. Just tick through and make sure we are in the correct state at the correct time:
      do
      {
         ExampleStateName currentStateEnum = stateMachine.getCurrentStateEnum();
         if (time.getValue() < 1.01)
         {
            assertEquals(ExampleStateName.Starting, currentStateEnum);
            assertFalse(stateMachine.inCurrentStateForDuration(1.01));
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

         if (stateMachine.timeInCurrentState() < 0.001)
         {
            assertTrue(currentState.didTransitionIntoAction);
            assertFalse(currentState.didAction);
            assertFalse(currentState.didTransitionOutOfAction);
         }

         boolean wontTransition = stateMachine.timeInCurrentState() < 1.0;

         stateMachine.doAction();
         stateMachine.checkTransitionConditions();
         time.setValue(time.getValue() + 0.01);

         double timeInCurrentState = stateMachine.timeInCurrentState();
         assertTrue(stateMachine.inCurrentStateForDuration(timeInCurrentState));
         assertFalse(stateMachine.inCurrentStateForDuration(timeInCurrentState + 1e-5));

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructionOfSimpleEventBasedStateMachine()
   {
      // This shows a quick and easy way to make a simple state machine without using the complex machinery for a more complex state machine.
      // This one goes through states Start, StateOne, StateTwo, and Stop.
      // It transitions based on calling the transitionToDefaultNextState() method in the State...

      YoVariableRegistry registry = new YoVariableRegistry("registry");
      SettableDoubleProvider time = new SettableDoubleProvider();
      StateMachine<ExampleStateName> stateMachine = new StateMachine<ExampleStateName>("stateMachine", "switchTime", ExampleStateName.class, time, registry);

      SimpleExampleState startingState = new SimpleExampleState(ExampleStateName.Starting);
      startingState.setDefaultNextState(ExampleStateName.StateOne);
      stateMachine.addState(startingState);

      SimpleExampleState stateOne = new SimpleExampleState(ExampleStateName.StateOne);
      stateOne.setDefaultNextState(ExampleStateName.StateTwo);
      stateMachine.addState(stateOne);

      SimpleExampleState stateTwo = new SimpleExampleState(ExampleStateName.StateTwo);
      stateTwo.setDefaultNextState(ExampleStateName.Stopped);
      stateMachine.addState(stateTwo);

      SimpleExampleState stoppedState = new SimpleExampleState(ExampleStateName.Stopped);
      stateMachine.addState(stoppedState);

      stateMachine.setCurrentState(ExampleStateName.Starting);

      // Some simple tests. Just tick through and make sure we transition through the states properly:

      assertEquals(startingState, stateMachine.getState(ExampleStateName.Starting));
      assertEquals(stateOne, stateMachine.getState(ExampleStateName.StateOne));
      assertEquals(stateTwo, stateMachine.getState(ExampleStateName.StateTwo));
      assertEquals(stoppedState, stateMachine.getState(ExampleStateName.Stopped));

      assertEquals("switchTime", stateMachine.getSwitchTimeName());
      assertEquals("stateMachine", stateMachine.getStateYoVariableName());

      assertTrue(stateMachine.isCurrentState(ExampleStateName.Starting));

      assertEquals(startingState, stateMachine.getState(ExampleStateName.Starting));
      assertEquals(stateOne, stateMachine.getState(ExampleStateName.StateOne));
      assertEquals(stateTwo, stateMachine.getState(ExampleStateName.StateTwo));
      assertEquals(stoppedState, stateMachine.getState(ExampleStateName.Stopped));

      RememberStateChangedListener listener = new RememberStateChangedListener();
      stateMachine.attachStateChangedListener(listener);

      RememberStateChangedListener listenerTwo = new RememberStateChangedListener();
      stateMachine.attachStateChangedListener(listenerTwo);

      ExampleStateName currentStateEnum = stateMachine.getCurrentStateEnum();
      SimpleExampleState currentState = (SimpleExampleState) stateMachine.getCurrentState();

      assertEquals(currentState, startingState);
      assertEquals(currentStateEnum, ExampleStateName.Starting);

      // Not sure if this should be Starting or null here. Hmm...
      assertEquals(currentState.getPreviousState(), startingState);

      assertTrue(currentState.didTransitionIntoAction);
      assertFalse(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      time.setValue(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      assertNull(listener.oldState);
      assertNull(listener.newState);
      assertEquals(0.0, listener.time, 1e-7);

      assertTrue(stateMachine.isCurrentState(ExampleStateName.Starting));
      currentState.transitionToDefaultNextState();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertTrue(stateMachine.isCurrentState(ExampleStateName.StateOne));

      assertEquals(startingState, listener.oldState);
      assertEquals(stateOne, listener.newState);
      assertEquals(time.getValue(), listener.time, 1e-7);

      time.setValue(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateEnum();
      currentState = (SimpleExampleState) stateMachine.getCurrentState();

      assertEquals(currentState, stateOne);
      assertEquals(currentStateEnum, ExampleStateName.StateOne);
      assertEquals(currentState.getPreviousState(), startingState);

      assertTrue(currentState.didTransitionIntoAction);
      assertFalse(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      assertEquals(startingState, listener.oldState);
      assertEquals(stateOne, listener.newState);

      assertTrue(stateMachine.isCurrentState(ExampleStateName.StateOne));

      currentState.transitionToDefaultNextState();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertFalse(stateMachine.isCurrentState(ExampleStateName.StateOne));
      assertTrue(stateMachine.isCurrentState(ExampleStateName.StateTwo));

      assertEquals(stateOne, listener.oldState);
      assertEquals(stateTwo, listener.newState);
      assertEquals(stateOne, listenerTwo.oldState);
      assertEquals(stateTwo, listenerTwo.newState);
      assertEquals(time.getValue(), listener.time, 1e-7);

      time.setValue(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateEnum();
      currentState = (SimpleExampleState) stateMachine.getCurrentState();

      assertTrue(stateMachine.isCurrentState(ExampleStateName.StateTwo));
      assertEquals(currentState, stateTwo);
      assertEquals(currentStateEnum, ExampleStateName.StateTwo);
      assertEquals(currentState.getPreviousState(), stateOne);

      assertTrue(currentState.didTransitionIntoAction);
      assertFalse(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      time.setValue(time.getValue() + 0.01);

      assertEquals(stateTwo, stateMachine.getCurrentState());
      assertEquals(ExampleStateName.StateTwo, stateMachine.getCurrentStateEnum());

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      assertEquals(stateOne, listener.oldState);
      assertEquals(stateTwo, listener.newState);

      assertTrue(stateMachine.isCurrentState(ExampleStateName.StateTwo));

      currentState.transitionToDefaultNextState();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertTrue(stateMachine.isCurrentState(ExampleStateName.Stopped));

      assertEquals(stateTwo, listener.oldState);
      assertEquals(stoppedState, listener.newState);
      assertEquals(time.getValue(), listener.time, 1e-7);

      time.setValue(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateEnum();
      currentState = (SimpleExampleState) stateMachine.getCurrentState();

      assertEquals(currentState, stoppedState);
      assertEquals(currentStateEnum, ExampleStateName.Stopped);
      assertEquals(currentState.getPreviousState(), stateTwo);

      assertTrue(currentState.didTransitionIntoAction);
      assertFalse(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      time.setValue(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeStateMachineExceptions()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      SettableDoubleProvider time = new SettableDoubleProvider();
      StateMachine<ExampleStateName> stateMachine = new StateMachine<ExampleStateName>("stateMachine", "switchTime", ExampleStateName.class, time, registry);

      assertNull(stateMachine.getState(ExampleStateName.Starting));

      SimpleExampleState startingState = new SimpleExampleState(ExampleStateName.Starting);
      startingState.setDefaultNextState(ExampleStateName.StateOne);
      stateMachine.addState(startingState);

      try
      {
         stateMachine.addState(startingState);
         fail("Repeat State");
      }
      catch(RuntimeException e)
      {
         assertEquals("Duplicate state enums, name: Starting, already in use.", e.getMessage());
      }

      try
      {
         stateMachine.setCurrentState(ExampleStateName.StateOne);
         fail("State not added yet");
      }
      catch(RuntimeException e)
      {
         assertEquals("Need to add state StateOne to the state machine. Can't transition into the state unless it is added!", e.getMessage());
      }

      SimpleExampleState stateOne = new SimpleExampleState(ExampleStateName.StateOne);
      stateOne.setDefaultNextState(ExampleStateName.StateTwo);
      stateMachine.addState(stateOne);

      try
      {
         stateOne.setDefaultNextState(ExampleStateName.StateTwo);
         fail("Already set default next state.");
      }
      catch(RuntimeException e)
      {
         assertEquals("Have already set default next state for StateOne", e.getMessage());
      }
      try
      {
         stateMachine.addState(stateOne);
         fail("Repeat State");
      }
      catch(RuntimeException e)
      {
         assertEquals("Duplicate state enums, name: StateOne, already in use.", e.getMessage());
      }

      try
      {
         stateMachine.setCurrentState(ExampleStateName.Stopped);
         fail("Cannot set current state that is not added to the state machine.");
      }
      catch(RuntimeException e)
      {
         assertEquals("Need to add state Stopped to the state machine. Can't transition into the state unless it is added!", e.getMessage());
      }

      SimpleExampleState stateTwo = new SimpleExampleState(ExampleStateName.StateTwo);
      stateMachine.addState(stateTwo);
      stateMachine.setCurrentState(ExampleStateName.StateTwo);

      try
      {
         stateTwo.transitionToDefaultNextState();
         stateMachine.checkTransitionConditions();
         fail("DefaultNextState not set");
      }
      catch(RuntimeException e)
      {
         assertEquals("DefaultNextState was not set for currentState=StateTwo: ()", e.getMessage());
      }

      EnumYoVariable<ExampleStateName> stateYoVariable = stateMachine.getStateYoVariable();

      try
      {
         stateYoVariable.set(null);
         fail("Can't set null!");
      }
      catch(Exception e)
      {
         assertEquals("Setting EnumYoVariable stateMachine to null. Must set allowNullValue to true in the constructor if you ever want to set it to null.", e.getMessage());
      }
   }

   private class SimpleExampleState extends State<ExampleStateName>
   {
      public boolean didTransitionIntoAction = false;
      public boolean didAction = false;
      public boolean didTransitionOutOfAction = false;

      public SimpleExampleState(ExampleStateName stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         didAction = true;
      }

      @Override
      public void doTransitionIntoAction()
      {
         didTransitionIntoAction = true;
      }

      @Override
      public void doTransitionOutOfAction()
      {
         didTransitionOutOfAction = true;
      }

   }

   private enum ExampleStateName
   {
      Starting, StateOne, StateTwo, Stopped
   }

   private class RememberStateChangedListener implements StateChangedListener<ExampleStateName>
   {
      public State<ExampleStateName> oldState;
      public State<ExampleStateName> newState;
      public double time;

      @Override
      public void stateChanged(State<ExampleStateName> oldState, State<ExampleStateName> newState, double time)
      {
         this.oldState = oldState;
         this.newState = newState;
         this.time = time;
      };
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(StateMachineExampleOneTest.class);
   }

}
