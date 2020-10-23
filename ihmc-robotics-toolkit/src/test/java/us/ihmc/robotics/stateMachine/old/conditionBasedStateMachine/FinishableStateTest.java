package us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FinishableStateTest
{

   @Test
   public void testExampleStateMachineWithFinishableStates()
   {
      YoRegistry registry = new YoRegistry("registry");
      YoDouble time = new YoDouble("time", registry);

      StateMachine<StateEnum> stateMachine = new StateMachine<StateEnum>("stateMachine", "switchTime", StateEnum.class, time, registry);

      ExampleFinishableState stateOne = new ExampleFinishableState(StateEnum.ONE);
      stateOne.addDoneWithStateTransition(StateEnum.TWO);
      stateMachine.addState(stateOne);

      ExampleFinishableState stateTwo = new ExampleFinishableState(StateEnum.TWO);
      stateTwo.addDoneWithStateTransition(StateEnum.THREE);
      stateMachine.addState(stateTwo);

      ExampleFinishableState stateThree = new ExampleFinishableState(StateEnum.THREE);
      stateThree.addDoneWithStateTransition(StateEnum.ONE);
      stateMachine.addState(stateThree);

      stateMachine.setCurrentState(StateEnum.ONE);

      // Run through some tests:
      assertTrue(stateOne.inState);
      assertFalse(stateOne.didAction);
      assertFalse(stateOne.didTransitionOutOfAction);

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertEquals(stateMachine.getCurrentState(), stateOne);
      assertTrue(stateOne.didAction);
      assertFalse(stateOne.didTransitionOutOfAction);

      stateOne.setIsDone(true);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertTrue(stateOne.didTransitionOutOfAction);
      assertTrue(stateTwo.inState);
      assertFalse(stateTwo.didAction);
      assertFalse(stateTwo.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateTwo);

      stateTwo.setIsDone(true);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertTrue(stateTwo.didTransitionOutOfAction);
      assertTrue(stateThree.inState);
      assertFalse(stateThree.didAction);
      assertFalse(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateThree);

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      assertTrue(stateThree.inState);
      assertTrue(stateThree.didAction);
      assertFalse(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateThree);

      stateThree.setIsDone(true);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      assertTrue(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateOne);
   }

   private class ExampleFinishableState extends FinishableState<StateEnum>
   {
      private boolean isDone = false;
      public boolean inState = false;
      public boolean didAction = false;
      public boolean didTransitionOutOfAction = false;

      public ExampleFinishableState(StateEnum stateEnum)
      {
         super(stateEnum);
      }

      public void setIsDone(boolean isDone)
      {
         this.isDone = isDone;
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }

      @Override
      public void doAction()
      {
         didAction = true;
      }

      @Override
      public void doTransitionIntoAction()
      {
         inState = true;
      }

      @Override
      public void doTransitionOutOfAction()
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
