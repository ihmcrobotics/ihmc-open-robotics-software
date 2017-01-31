package us.ihmc.robotics.stateMachines;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.*;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.MutationTestingTools;

public class StateMachineExampleTwoTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComplexStateMachineExample()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      final IntegerYoVariable ticksInStateOne = new IntegerYoVariable("ticksInStateOne", registry);
      final IntegerYoVariable ticksInStateTwo = new IntegerYoVariable("ticksInStateTwo", registry);
      final IntegerYoVariable ticksInStateThree = new IntegerYoVariable("ticksInStateThree", registry);
      final IntegerYoVariable ticksInStateFour = new IntegerYoVariable("ticksInStateFour", registry);

      final BooleanYoVariable inStateOne = new BooleanYoVariable("inStateOne", registry);
      final BooleanYoVariable inStateTwo = new BooleanYoVariable("inStateTwo", registry);
      final BooleanYoVariable inStateThree = new BooleanYoVariable("inStateThree", registry);
      final BooleanYoVariable inStateFour = new BooleanYoVariable("inStateFour", registry);

      final BooleanYoVariable didTransitionIntoStateOne = new BooleanYoVariable("didTransitionIntoStateOne", registry);
      final BooleanYoVariable didTransitionIntoStateTwo = new BooleanYoVariable("didTransitionIntoStateTwo", registry);
      final BooleanYoVariable didTransitionIntoStateThree = new BooleanYoVariable("didTransitionIntoStateThree", registry);
      final BooleanYoVariable didTransitionIntoStateFour = new BooleanYoVariable("didTransitionIntoStateFour", registry);

      final BooleanYoVariable didTransitionOutOfStateOne = new BooleanYoVariable("didTransitionOutOfStateOne", registry);
      final BooleanYoVariable didTransitionOutOfStateTwo = new BooleanYoVariable("didTransitionOutOfStateTwo", registry);
      final BooleanYoVariable didTransitionOutOfStateThree = new BooleanYoVariable("didTransitionOutOfStateThree", registry);
      final BooleanYoVariable didTransitionOutOfStateFour = new BooleanYoVariable("didTransitionOutOfStateFour", registry);

      final BooleanYoVariable transitionFromOneToFour = new BooleanYoVariable("transitionFromOneToFour", registry);
      final BooleanYoVariable transitionFromThreeToFour = new BooleanYoVariable("transitionFromThreeToFour", registry);
      final BooleanYoVariable transitionFromThreeToOne = new BooleanYoVariable("transitionFromThreeToOne", registry);

      final BooleanYoVariable threeToOneTransitionAction = new BooleanYoVariable("threeToOneTransitionAction", registry);

      DoubleYoVariable timeProvider = new DoubleYoVariable("time", registry);
      timeProvider.set(13.3);

      StateMachine<StateEnum> stateMachine = new StateMachine<StateEnum>("complexStateMachine", "switchTime", StateEnum.class, StateEnum.FOUR, timeProvider, registry);

      ExampleState stateOne = new ExampleState(StateEnum.ONE, inStateOne, didTransitionIntoStateOne, didTransitionOutOfStateOne, ticksInStateOne);
      ExampleState stateTwo = new ExampleState(StateEnum.TWO, inStateTwo, didTransitionIntoStateTwo, didTransitionOutOfStateTwo, ticksInStateTwo);
      ExampleState stateThree = new ExampleState(StateEnum.THREE, inStateThree, didTransitionIntoStateThree, didTransitionOutOfStateThree, ticksInStateThree);
      ExampleState stateFour = new ExampleState(StateEnum.FOUR, inStateFour, didTransitionIntoStateFour, didTransitionOutOfStateFour, ticksInStateFour);

      stateOne.setDefaultNextState(StateEnum.TWO); // Do this transition by calling stateOne.transitionToDefaultNextState();
      stateOne.addStateTransition(StateEnum.FOUR, new ExampleStateTransitionCondition(transitionFromOneToFour));

      stateTwo.addDelayBasedStateTransition(StateEnum.THREE, 1.0);

      StateTransitionCondition threeToFourCondition = new ExampleStateTransitionCondition(transitionFromThreeToFour);
      stateThree.addStateTransition(StateEnum.FOUR, threeToFourCondition);

      StateTransitionCondition stateTransitionCondition = new ExampleStateTransitionCondition(transitionFromThreeToOne);
      StateTransitionAction stateTransitionAction = new ExampleStateTransitionAction(threeToOneTransitionAction);
      StateTransition<StateEnum> stateTransition = new StateTransition<StateEnum>(StateEnum.ONE, stateTransitionCondition, stateTransitionAction);

      stateThree.addStateTransition(stateTransition);

      stateThree.addImmediateStateTransition(StateEnum.THREE); // Transition into itself (and call the transitionIntoAction() each time.


      ArrayList<StateTransition<StateEnum>> stateTransitions = stateThree.getStateTransitions();
      assertEquals(3, stateTransitions.size());
      assertEquals(StateEnum.FOUR, stateTransitions.get(0).getNextStateEnum());
      assertEquals(StateEnum.ONE, stateTransitions.get(1).getNextStateEnum());
      assertEquals(StateEnum.THREE, stateTransitions.get(2).getNextStateEnum());

      stateFour.addImmediateStateTransition(StateEnum.THREE);

      stateMachine.addState(stateOne);
      stateMachine.addState(stateTwo);
      stateMachine.addState(stateThree);
      stateMachine.addState(stateFour);

      assertEquals(stateFour, stateMachine.getCurrentState());
      assertEquals(StateEnum.FOUR, stateMachine.getCurrentStateEnum());

      assertEquals(null, stateMachine.getPreviousStateEnum());
      assertEquals(null, stateMachine.getPreviousState());
      assertNull(stateOne.getPreviousState());
      assertFalse(inStateOne.getBooleanValue());
      assertFalse(didTransitionIntoStateOne.getBooleanValue());
      assertFalse(didTransitionOutOfStateOne.getBooleanValue());
      assertEquals(0, ticksInStateOne.getIntegerValue());

      assertNull(stateFour.getPreviousState());
      assertFalse(inStateFour.getBooleanValue());
      assertFalse(didTransitionIntoStateFour.getBooleanValue());
      assertFalse(didTransitionOutOfStateFour.getBooleanValue());
      assertEquals(0, ticksInStateFour.getIntegerValue());

      stateMachine.setCurrentState(StateEnum.ONE);
      assertTrue(inStateOne.getBooleanValue());
      assertEquals(stateOne, stateMachine.getCurrentState());
      assertEquals(StateEnum.ONE, stateMachine.getCurrentStateEnum());
      assertTrue(didTransitionIntoStateOne.getBooleanValue());
      assertFalse(didTransitionOutOfStateOne.getBooleanValue());
      assertEquals(0, ticksInStateOne.getIntegerValue()); //setCurrentState should do the transitionInto, but not the doAction().

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertTrue(inStateOne.getBooleanValue());
      assertTrue(didTransitionIntoStateOne.getBooleanValue());
      assertFalse(didTransitionOutOfStateOne.getBooleanValue());
      assertEquals(1, ticksInStateOne.getIntegerValue());

      stateOne.transitionToDefaultNextState();
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertFalse(inStateOne.getBooleanValue());
      assertTrue(didTransitionIntoStateOne.getBooleanValue());
      assertTrue(didTransitionOutOfStateOne.getBooleanValue());
      assertEquals(2, ticksInStateOne.getIntegerValue());

      assertEquals(StateEnum.ONE, stateMachine.getPreviousStateEnum());
      assertEquals(stateOne, stateMachine.getPreviousState());
      assertEquals(stateOne, stateTwo.getPreviousState());
      assertTrue(inStateTwo.getBooleanValue());
      assertTrue(didTransitionIntoStateTwo.getBooleanValue());
      assertFalse(didTransitionOutOfStateTwo.getBooleanValue());
      assertEquals(0, ticksInStateTwo.getIntegerValue());

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      assertTrue(inStateTwo.getBooleanValue());
      assertTrue(didTransitionIntoStateTwo.getBooleanValue());
      assertFalse(didTransitionOutOfStateTwo.getBooleanValue());
      assertEquals(1, ticksInStateTwo.getIntegerValue());
      assertFalse(didTransitionIntoStateThree.getBooleanValue());

      timeProvider.add(1.01);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      assertFalse(inStateTwo.getBooleanValue());
      assertTrue(didTransitionIntoStateTwo.getBooleanValue());
      assertTrue(didTransitionOutOfStateTwo.getBooleanValue());
      assertEquals(2, ticksInStateTwo.getIntegerValue());

      assertEquals(StateEnum.TWO, stateMachine.getPreviousStateEnum());
      assertEquals(stateTwo, stateMachine.getPreviousState());
      assertEquals(stateTwo, stateThree.getPreviousState());
      assertTrue(inStateThree.getBooleanValue());
      assertTrue(didTransitionIntoStateThree.getBooleanValue());
      assertFalse(didTransitionOutOfStateThree.getBooleanValue());
      assertEquals(0, ticksInStateThree.getIntegerValue());

      didTransitionIntoStateThree.set(false);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      assertTrue(inStateThree.getBooleanValue());
      assertTrue(didTransitionIntoStateThree.getBooleanValue());
      assertTrue(didTransitionOutOfStateThree.getBooleanValue());
      assertEquals(1, ticksInStateThree.getIntegerValue());

      didTransitionOutOfStateThree.set(false);
      didTransitionIntoStateThree.set(false);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      assertTrue(inStateThree.getBooleanValue());
      assertTrue(didTransitionIntoStateThree.getBooleanValue());
      assertTrue(didTransitionOutOfStateThree.getBooleanValue());
      assertEquals(2, ticksInStateThree.getIntegerValue());

      transitionFromThreeToFour.set(true);
      didTransitionIntoStateThree.set(false);
      didTransitionOutOfStateThree.set(false);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      assertFalse(inStateThree.getBooleanValue());
      assertFalse(didTransitionIntoStateThree.getBooleanValue());
      assertTrue(didTransitionOutOfStateThree.getBooleanValue());
      assertEquals(3, ticksInStateThree.getIntegerValue());

      assertEquals(StateEnum.THREE, stateMachine.getPreviousStateEnum());
      assertEquals(stateThree, stateMachine.getPreviousState());
      assertEquals(stateThree, stateFour.getPreviousState());
      assertTrue(inStateFour.getBooleanValue());
      assertTrue(didTransitionIntoStateFour.getBooleanValue());
      assertFalse(didTransitionOutOfStateFour.getBooleanValue());
      assertEquals(0, ticksInStateFour.getIntegerValue());

      didTransitionIntoStateThree.set(false);
      didTransitionOutOfStateThree.set(false);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertFalse(inStateFour.getBooleanValue());
      assertTrue(didTransitionIntoStateFour.getBooleanValue());
      assertTrue(didTransitionOutOfStateFour.getBooleanValue());
      assertEquals(1, ticksInStateFour.getIntegerValue());

      assertEquals(StateEnum.FOUR, stateMachine.getPreviousStateEnum());
      assertEquals(stateFour, stateMachine.getPreviousState());
      assertEquals(stateFour, stateThree.getPreviousState());
      assertTrue(inStateThree.getBooleanValue());
      assertTrue(didTransitionIntoStateThree.getBooleanValue());
      assertFalse(didTransitionOutOfStateThree.getBooleanValue());
      assertEquals(3, ticksInStateThree.getIntegerValue());

      transitionFromThreeToOne.set(true);
      transitionFromThreeToFour.set(false);
      didTransitionIntoStateOne.set(false);
      didTransitionOutOfStateOne.set(false);

      assertFalse(threeToOneTransitionAction.getBooleanValue());

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertTrue(threeToOneTransitionAction.getBooleanValue());
      assertTrue(inStateOne.getBooleanValue());
      assertTrue(didTransitionIntoStateOne.getBooleanValue());
      assertFalse(didTransitionOutOfStateOne.getBooleanValue());
      assertEquals(2, ticksInStateOne.getIntegerValue());

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertEquals(StateEnum.THREE, stateMachine.getPreviousStateEnum());
      assertEquals(stateThree, stateMachine.getPreviousState());
      assertEquals(stateThree, stateOne.getPreviousState());
      assertTrue(inStateOne.getBooleanValue());
      assertTrue(didTransitionIntoStateOne.getBooleanValue());
      assertFalse(didTransitionOutOfStateOne.getBooleanValue());
      assertEquals(3, ticksInStateOne.getIntegerValue());

      transitionFromOneToFour.set(true);
      didTransitionIntoStateFour.set(false);
      didTransitionOutOfStateFour.set(false);
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();

      assertFalse(inStateOne.getBooleanValue());
      assertTrue(didTransitionIntoStateOne.getBooleanValue());
      assertTrue(didTransitionOutOfStateOne.getBooleanValue());
      assertEquals(4, ticksInStateOne.getIntegerValue());

      assertEquals(stateOne, stateFour.getPreviousState());
      assertEquals(stateFour, stateMachine.getCurrentState());
      assertTrue(inStateFour.getBooleanValue());
      assertTrue(didTransitionIntoStateFour.getBooleanValue());
      assertFalse(didTransitionOutOfStateFour.getBooleanValue());
      assertEquals(1, ticksInStateFour.getIntegerValue());

      assertEquals("ONE: (FOUR)", stateOne.toString());
      assertEquals("TWO: (THREE)", stateTwo.toString());
      assertEquals("THREE: (FOUR, ONE, THREE)", stateThree.toString());
      assertEquals("FOUR: (THREE)", stateFour.toString());

      assertEquals("State Machine:\nONE: (FOUR)\nTWO: (THREE)\nTHREE: (FOUR, ONE, THREE)\nFOUR: (THREE)\n", stateMachine.toString());
   }

   private class ExampleStateTransitionCondition implements StateTransitionCondition
   {
      private final BooleanYoVariable transitionVariable;

      public ExampleStateTransitionCondition(BooleanYoVariable transitionVariable)
      {
         this.transitionVariable = transitionVariable;
      }

      @Override
      public boolean checkCondition()
      {
         return transitionVariable.getBooleanValue();
      }

   }

   private class ExampleStateTransitionAction implements StateTransitionAction
   {
      private final BooleanYoVariable transitionVariable;

      public ExampleStateTransitionAction(BooleanYoVariable transitionVariable)
      {
         this.transitionVariable = transitionVariable;
      }

      @Override
      public void doTransitionAction()
      {
         transitionVariable.set(true);
      }
   }

   private class ExampleState extends State<StateEnum>
   {
      private final BooleanYoVariable inState, didTransitionIntoState, didTransitionOutOfState;
      private final IntegerYoVariable ticksInState;

      public ExampleState(StateEnum stateEnum, BooleanYoVariable inState, BooleanYoVariable didTransitionIntoState, BooleanYoVariable didTransitionOutOfState, IntegerYoVariable ticksInState)
      {
         super(stateEnum);

         this.inState = inState;
         this.didTransitionIntoState = didTransitionIntoState;
         this.didTransitionOutOfState = didTransitionOutOfState;
         this.ticksInState = ticksInState;
      }

      @Override
      public void doAction()
      {
         ticksInState.increment();
      }

      @Override
      public void doTransitionIntoAction()
      {
         didTransitionIntoState.set(true);
         inState.set(true);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         didTransitionOutOfState.set(true);
         inState.set(false);
      }
   }

   private enum StateEnum
   {
      // Mixed up on purpose to make sure state creation ordering doesn't matter.
      TWO, FOUR, ONE, THREE
   }

   public static void main(String[] args)
   {
      String targetTests = StateMachineExampleTwoTest.class.getName();
      String targetClassesInSamePackage = MutationTestingTools.createClassSelectorStringFromTargetString(targetTests);
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
