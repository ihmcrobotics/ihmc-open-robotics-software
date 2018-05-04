package us.ihmc.robotics.stateMachine;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StateMachineTest
{
   private static final double EPSILON = 1e-7;
   private static final boolean VERBOSE = false;
   private final double INTO = 0.0;
   private final double ACTION = 0.1;
   private final double OUT_OF = 0.9;
   private final int MAX_NUMBER_OF_CALLS = 2;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testStateTransitions()
   {
      ArrayList<Double> listOfActions = new ArrayList<>();

      SimpleState[] arrayOfStates = new SimpleState[States.values().length];

      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble time = new YoDouble("time", registry);

      StateMachineFactory<States, SimpleState> factory = new StateMachineFactory<>(States.class);
      factory.setNamePrefix("bellopp");
      factory.setRegistry(registry);
      factory.buildYoClock(time);

      for (int i = 0; i < arrayOfStates.length; i++)
      {
         arrayOfStates[i] = new SimpleState(States.values()[i], i, listOfActions);
      }

      for (int i = 0; i < arrayOfStates.length - 1; i++)
      {
         factory.addStateAndDoneTransition(arrayOfStates[i].getStateEnum(), arrayOfStates[i], arrayOfStates[i + 1].getStateEnum());
      }

      factory.addState(arrayOfStates[arrayOfStates.length - 1].getStateEnum(), arrayOfStates[arrayOfStates.length - 1]);

      StateMachine<States, SimpleState> stateMachine = factory.build(arrayOfStates[0].getStateEnum());

      do
      {
         stateMachine.doActionAndTransition();
         time.add(0.01);
      }
      while (!stateMachine.getCurrentStateKey().equals(States.values()[States.values().length - 1]));

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

            assertTrue(didTransitionOutOf);
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
            assertTrue("actionCode = " + actionCode + ", ACTION = " + ACTION + ", OUT_OF = " + OUT_OF,
                       Math.abs(actionCode - ACTION) < EPSILON || Math.abs(actionCode - OUT_OF) < EPSILON);

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

   public class SimpleState implements State
   {
      private final int stateID;
      private ArrayList<Double> listOfActions;

      private int callCounter = 1;
      private final States stateEnum;
      private boolean isDone = false;

      public SimpleState(States stateEnum, int stateID, ArrayList<Double> listOfActions)
      {
         this.stateEnum = stateEnum;
         this.listOfActions = listOfActions;
         this.stateID = stateID;
      }

      @Override
      public void doAction(double timeInState)
      {
         double actionID = stateID + ACTION;
         listOfActions.add(actionID);

         if (callCounter >= MAX_NUMBER_OF_CALLS)
            isDone = true;
         ;

         callCounter++;
      }

      @Override
      public void onEntry()
      {
         double actionID = stateID + INTO;
         listOfActions.add(actionID);
      }

      @Override
      public void onExit()
      {
         double actionID = stateID + OUT_OF;
         listOfActions.add(actionID);
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone;
      }

      public States getStateEnum()
      {
         return stateEnum;
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
