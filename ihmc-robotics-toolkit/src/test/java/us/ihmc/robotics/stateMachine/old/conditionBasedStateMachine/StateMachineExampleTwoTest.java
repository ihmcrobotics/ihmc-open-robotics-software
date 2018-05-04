package us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionAction;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class StateMachineExampleTwoTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComplexStateMachineExample()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      final YoInteger ticksInStateOne = new YoInteger("ticksInStateOne", registry);
      final YoInteger ticksInStateTwo = new YoInteger("ticksInStateTwo", registry);
      final YoInteger ticksInStateThree = new YoInteger("ticksInStateThree", registry);
      final YoInteger ticksInStateFour = new YoInteger("ticksInStateFour", registry);

      final YoBoolean inStateOne = new YoBoolean("inStateOne", registry);
      final YoBoolean inStateTwo = new YoBoolean("inStateTwo", registry);
      final YoBoolean inStateThree = new YoBoolean("inStateThree", registry);
      final YoBoolean inStateFour = new YoBoolean("inStateFour", registry);

      final YoBoolean didTransitionIntoStateOne = new YoBoolean("didTransitionIntoStateOne", registry);
      final YoBoolean didTransitionIntoStateTwo = new YoBoolean("didTransitionIntoStateTwo", registry);
      final YoBoolean didTransitionIntoStateThree = new YoBoolean("didTransitionIntoStateThree", registry);
      final YoBoolean didTransitionIntoStateFour = new YoBoolean("didTransitionIntoStateFour", registry);

      final YoBoolean didTransitionOutOfStateOne = new YoBoolean("didTransitionOutOfStateOne", registry);
      final YoBoolean didTransitionOutOfStateTwo = new YoBoolean("didTransitionOutOfStateTwo", registry);
      final YoBoolean didTransitionOutOfStateThree = new YoBoolean("didTransitionOutOfStateThree", registry);
      final YoBoolean didTransitionOutOfStateFour = new YoBoolean("didTransitionOutOfStateFour", registry);

      final YoBoolean transitionFromOneToFour = new YoBoolean("transitionFromOneToFour", registry);
      final YoBoolean transitionFromThreeToFour = new YoBoolean("transitionFromThreeToFour", registry);
      final YoBoolean transitionFromThreeToOne = new YoBoolean("transitionFromThreeToOne", registry);

      final YoBoolean threeToOneTransitionAction = new YoBoolean("threeToOneTransitionAction", registry);

      YoDouble timeProvider = new YoDouble("time", registry);
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
      private final YoBoolean transitionVariable;

      public ExampleStateTransitionCondition(YoBoolean transitionVariable)
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
      private final YoBoolean transitionVariable;

      public ExampleStateTransitionAction(YoBoolean transitionVariable)
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
      private final YoBoolean inState, didTransitionIntoState, didTransitionOutOfState;
      private final YoInteger ticksInState;

      public ExampleState(StateEnum stateEnum, YoBoolean inState, YoBoolean didTransitionIntoState, YoBoolean didTransitionOutOfState, YoInteger ticksInState)
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
      MutationTestFacilitator.facilitateMutationTestForPackage(StateMachineExampleTwoTest.class);
   }
}
