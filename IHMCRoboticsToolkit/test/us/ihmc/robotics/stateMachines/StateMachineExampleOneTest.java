package us.ihmc.robotics.stateMachines;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class StateMachineExampleOneTest
{
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructionOfSimpleTimeBasedStateMachine()
   {
      // This shows a quick and easy way to make a simple state machine without using the complex machinery for a more complex state machine.
      // This one goes through states Start, StateOne, StateTwo, and Stop.
      // It transitions based on time.
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      SettableDoubleProvider time = new SettableDoubleProvider();
      StateMachine<ExampleStateName> stateMachine = new StateMachine<ExampleStateName>("stateMachine", "switchTime", ExampleStateName.class, time, registry);

      SimpleState startingState = new SimpleState(ExampleStateName.Starting);
      startingState.addDelayBasedStateTransition(ExampleStateName.StateOne, 1.0);
      stateMachine.addState(startingState);

      SimpleState stateOne = new SimpleState(ExampleStateName.StateOne);
      stateOne.addDelayBasedStateTransition(ExampleStateName.StateTwo, 1.0);
      stateMachine.addState(stateOne);

      SimpleState stateTwo = new SimpleState(ExampleStateName.StateTwo);
      stateTwo.addDelayBasedStateTransition(ExampleStateName.Stopped, 1.0);
      stateMachine.addState(stateTwo);

      SimpleState stoppedState = new SimpleState(ExampleStateName.Stopped);
      stateMachine.addState(stoppedState);

      stateMachine.setCurrentState(ExampleStateName.Starting);

      // Some simple tests. Just tick through and make sure we are in the correct state at the correct time:
      do
      {
         ExampleStateName currentStateEnum = stateMachine.getCurrentStateEnum();
         if (time.getValue() < 1.01)
            assertEquals(ExampleStateName.Starting, currentStateEnum);
         else if (time.getValue() < 2.01)
            assertEquals(ExampleStateName.StateOne, currentStateEnum);
         else if (time.getValue() < 3.01)
            assertEquals(ExampleStateName.StateTwo, currentStateEnum);
         else
            assertEquals(ExampleStateName.Stopped, currentStateEnum);

         SimpleState currentState = (SimpleState) stateMachine.getCurrentState();

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

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructionOfSimpleEventBasedStateMachine()
   {
      // This shows a quick and easy way to make a simple state machine without using the complex machinery for a more complex state machine.
      // This one goes through states Start, StateOne, StateTwo, and Stop.
      // It transitions based on calling the transitionToDefaultNextState() method in the State...

      YoVariableRegistry registry = new YoVariableRegistry("registry");
      SettableDoubleProvider time = new SettableDoubleProvider();
      StateMachine<ExampleStateName> stateMachine = new StateMachine<ExampleStateName>("stateMachine", "switchTime", ExampleStateName.class, time, registry);

      SimpleState startingState = new SimpleState(ExampleStateName.Starting);
      startingState.setDefaultNextState(ExampleStateName.StateOne);
      stateMachine.addState(startingState);

      SimpleState stateOne = new SimpleState(ExampleStateName.StateOne);
      stateOne.setDefaultNextState(ExampleStateName.StateTwo);
      stateMachine.addState(stateOne);

      SimpleState stateTwo = new SimpleState(ExampleStateName.StateTwo);
      stateTwo.setDefaultNextState(ExampleStateName.Stopped);
      stateMachine.addState(stateTwo);

      SimpleState stoppedState = new SimpleState(ExampleStateName.Stopped);
      stateMachine.addState(stoppedState);

      stateMachine.setCurrentState(ExampleStateName.Starting);

      // Some simple tests. Just tick through and make sure we transition through the states properly:
      ExampleStateName currentStateEnum = stateMachine.getCurrentStateEnum();
      SimpleState currentState = (SimpleState) stateMachine.getCurrentState();

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

      currentState.transitionToDefaultNextState();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      time.setValue(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateEnum();
      currentState = (SimpleState) stateMachine.getCurrentState();

      assertEquals(currentState, stateOne);
      assertEquals(currentStateEnum, ExampleStateName.StateOne);
      assertEquals(currentState.getPreviousState(), startingState);

      assertTrue(currentState.didTransitionIntoAction);
      assertFalse(currentState.didAction);
      assertFalse(currentState.didTransitionOutOfAction);

      currentState.transitionToDefaultNextState();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      time.setValue(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateEnum();
      currentState = (SimpleState) stateMachine.getCurrentState();

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

      currentState.transitionToDefaultNextState();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      time.setValue(time.getValue() + 0.01);

      assertTrue(currentState.didTransitionIntoAction);
      assertTrue(currentState.didAction);
      assertTrue(currentState.didTransitionOutOfAction);

      currentStateEnum = stateMachine.getCurrentStateEnum();
      currentState = (SimpleState) stateMachine.getCurrentState();

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

   private class SimpleState extends State<ExampleStateName>
   {
      public boolean didTransitionIntoAction = false;
      public boolean didAction = false;
      public boolean didTransitionOutOfAction = false;

      public SimpleState(ExampleStateName stateEnum)
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

   public static void main(String[] args)
   {
      String targetTests = StateMachineExampleOneTest.class.getName();
      String targetClassesInSamePackage = MutationTestingTools.createClassSelectorStringFromTargetString(targetTests);
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }

}
