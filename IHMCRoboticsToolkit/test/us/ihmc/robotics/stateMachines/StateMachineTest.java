package us.ihmc.robotics.stateMachines;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class StateMachineTest
{
   private static final double EPSILON = 1e8;
   private static final boolean VERBOSE = false;
   private final double INTO = 0.0;
   private final double ACTION = 0.1;
   private final double OUT_OF = 0.9;
   private final int MAX_NUMBER_OF_CALLS = 2;

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testStateTransition()
   {
      ArrayList<Double> listOfActions = new ArrayList<Double>();

      SimpleState[] arrayOfStates = new SimpleState[States.values().length];

      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable time = new DoubleYoVariable("time", registry);

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
      }
      while (!stateMachine.getCurrentStateEnum().equals(States.values()[States.values().length - 1]));

      int currentState;
      boolean didTransitionInto = false;
      boolean didAction = false;
      boolean didTransitionOutOf = false;
      boolean initialized = false;
      int previousState = 0;

      for (Double value : listOfActions)
      {
         currentState = value.intValue();

         if (!initialized)
         {
            previousState = currentState;
         }

         if (previousState != currentState)
         {
            // check to make sure that it is a next state
            assertEquals(currentState, previousState + 1, EPSILON);

            assert (didTransitionOutOf);
            didTransitionOutOf = false;
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
            assertTrue(Math.abs(actionCode - ACTION) < EPSILON || Math.abs(actionCode - OUT_OF) < EPSILON);

            if (actionCode == OUT_OF)
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

      //    private final StateMachine stateMachine;
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
}
