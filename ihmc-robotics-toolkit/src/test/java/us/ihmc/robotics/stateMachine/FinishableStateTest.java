package us.ihmc.robotics.stateMachine;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FinishableStateTest
{
   @Test
   public void testExampleStateMachineWithFinishableStates()
   {
      StateMachineFactory<StateEnum, ExampleFinishableState> factory = new  StateMachineFactory<>(StateEnum.class);

      ExampleFinishableState stateOne = new ExampleFinishableState(StateEnum.ONE);
      ExampleFinishableState stateTwo = new ExampleFinishableState(StateEnum.TWO);
      ExampleFinishableState stateThree = new ExampleFinishableState(StateEnum.THREE);

      factory.addStateAndDoneTransition(StateEnum.ONE, stateOne, StateEnum.TWO);
      factory.addStateAndDoneTransition(StateEnum.TWO, stateTwo, StateEnum.THREE);
      factory.addStateAndDoneTransition(StateEnum.THREE, stateThree, StateEnum.ONE);
      factory.setNamePrefix("example");
      factory.setRegistry(new YoRegistry("dummy"));
      StateMachine<StateEnum, ExampleFinishableState> stateMachine = factory.build(StateEnum.ONE);

      stateMachine.doActionAndTransition();

      // Run through some tests:
      assertTrue(stateOne.inState);
      assertTrue(stateOne.didAction);
      assertFalse(stateOne.didTransitionOutOfAction);

      stateMachine.doActionAndTransition();

      assertEquals(stateMachine.getCurrentState(), stateOne);
      assertTrue(stateOne.didAction);
      assertFalse(stateOne.didTransitionOutOfAction);

      stateOne.setIsDone(true);
      stateMachine.doActionAndTransition();

      assertTrue(stateOne.didTransitionOutOfAction);
      assertTrue(stateTwo.inState);
      assertTrue(stateTwo.didAction);
      assertFalse(stateTwo.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateTwo);

      stateTwo.setIsDone(true);
      stateMachine.doActionAndTransition();

      assertTrue(stateTwo.didTransitionOutOfAction);
      assertTrue(stateThree.inState);
      assertTrue(stateThree.didAction);
      assertFalse(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateThree);

      stateMachine.doActionAndTransition();
      assertTrue(stateThree.inState);
      assertTrue(stateThree.didAction);
      assertFalse(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateThree);

      stateThree.setIsDone(true);
      stateMachine.doActionAndTransition();
      assertTrue(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateOne);
   }

   private class ExampleFinishableState implements State
   {
      private boolean isDone = false;
      public boolean inState = false;
      public boolean didAction = false;
      public boolean didTransitionOutOfAction = false;

      public ExampleFinishableState(StateEnum stateEnum)
      {
      }

      public void setIsDone(boolean isDone)
      {
         this.isDone = isDone;
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone;
      }

      @Override
      public void doAction(double timeInState)
      {
         didAction = true;
      }

      @Override
      public void onEntry()
      {
         inState = true;
      }

      @Override
      public void onExit()
      {
         didTransitionOutOfAction = true;
         inState = false;
      }

   }

   private enum StateEnum
   {
      ONE, TWO, THREE, FOUR;
   }
}
