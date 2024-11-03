package us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StateMachineTest
{
   private static final double EPSILON = 1e-7;
   private static final boolean VERBOSE = false;
   private final double INTO = 0.0;
   private final double ACTION = 0.1;
   private final double OUT_OF = 0.9;
   private final int MAX_NUMBER_OF_CALLS = 2;

	@Test
   public void testStateTransitions()
   {
      ArrayList<Double> listOfActions = new ArrayList<Double>();

      SimpleState[] arrayOfStates = new SimpleState[States.values().length];

      YoRegistry registry = new YoRegistry("registry");
      YoDouble time = new YoDouble("time", registry);

      StateMachine<States> stateMachine = new StateMachine<States>("stateMachine", "switchTime", States.class, time, registry);

      for (int i = 0; i < arrayOfStates.length; i++)
      {
         arrayOfStates[i] = new SimpleState(States.values()[i], i, listOfActions);
      }

      for (int i = 0; i < arrayOfStates.length - 1; i++)
      {
         arrayOfStates[i].setDefaultNextState(arrayOfStates[i + 1].getStateEnum());
      }

      for (State<States> state : arrayOfStates)
      {
         stateMachine.addState(state);
      }

      stateMachine.setCurrentState(arrayOfStates[0].getStateEnum());

      do
      {
         stateMachine.doAction();
         stateMachine.checkTransitionConditions();
         time.add(0.01);
      }
      while (!stateMachine.getCurrentStateEnum().equals(States.values()[States.values().length - 1]));

      int currentState;
      boolean didTransitionInto = false;
      boolean didAction = false;
      boolean didTransitionOutOf = true;
      boolean initialized = false;
      int previousState = 0;

      for (Double value : listOfActions)
      {
         currentState = value.intValue();

         if (!initialized)
         {
            previousState = currentState;
            initialized = true;
         }

         if (previousState != currentState)
         {
            // check to make sure that it is a next state
            assertEquals(currentState, previousState + 1, EPSILON);

            assertTrue (didTransitionOutOf);
            didTransitionOutOf = false;
            previousState = currentState;

         }

         double actionCode = value.doubleValue() - Math.floor(value.doubleValue());

         if (!didTransitionInto)
         {
            assertEquals(actionCode, INTO, 0.0);
            didTransitionInto = true;
         }
         else if (!didAction)
         {
            assertEquals(ACTION, actionCode, EPSILON);
            didAction = true;
         }
         else
         {
            assertTrue("actionCode = " + actionCode + ", ACTION = " + ACTION + ", OUT_OF = " + OUT_OF, Math.abs(actionCode - ACTION) < EPSILON || Math.abs(actionCode - OUT_OF) < EPSILON);

            if (Math.abs(actionCode - OUT_OF) < EPSILON)
            {
               didTransitionInto = false;
               didAction = false;
               didTransitionOutOf = true;
            }
         }

         if (VERBOSE)
            System.out.println(value);
      }

   }

   public class SimpleState extends State<States>
   {
      private final int stateID;
      private ArrayList<Double> listOfActions;

      private int callCounter = 1;

      public SimpleState(States stateEnum, int stateID, ArrayList<Double> listOfActions)
      {
         super(stateEnum);
         this.listOfActions = listOfActions;
         this.stateID = stateID;
      }

      public void doAction()
      {
         double actionID = stateID + ACTION;
         listOfActions.add(actionID);

         if (callCounter >= MAX_NUMBER_OF_CALLS)
            this.transitionToDefaultNextState();

         callCounter++;
      }

      public void doTransitionIntoAction()
      {
         double actionID = stateID + INTO;
         listOfActions.add(actionID);
      }

      public void doTransitionOutOfAction()
      {
         double actionID = stateID + OUT_OF;
         listOfActions.add(actionID);
      }
   }

   private enum States
   {
      ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX
   }


   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(StateMachineTest.class);
   }
}
