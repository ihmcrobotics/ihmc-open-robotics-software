package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateChangeRecorder;


public class StateVisitedTwiceSimulationDoneCriterion implements SimulationDoneCriterion
{
   private final StateChangeRecorder stateChangeRecorder;
   private final ArrayList<State> exceptions = new ArrayList<State>();
   private State stateVisitedTwice;
   private boolean isEnabled = true;

   public StateVisitedTwiceSimulationDoneCriterion(StateChangeRecorder stateChangeRecorder)
   {
      this.stateChangeRecorder = stateChangeRecorder;
   }

   public boolean isSimulationDone()
   {
      if (isEnabled)
      {
         HashMap<State, ArrayList<Double>> statesAndSwitchTimes = stateChangeRecorder.getStatesAndSwitchTimes();

         for (State state : exceptions)
         {
            statesAndSwitchTimes.remove(state);
         }

         for (State state : statesAndSwitchTimes.keySet())
         {
            if (statesAndSwitchTimes.get(state).size() > 1)
            {
               stateVisitedTwice = state;

               return true;
            }
         }
      }

      return false;
   }

   public State getStateVisitedTwice()
   {
      return stateVisitedTwice;
   }

   public void addException(State exceptionState)
   {
      this.exceptions.add(exceptionState);
   }

   public void enable()
   {
      isEnabled = true;
   }

   public void disable()
   {
      isEnabled = false;
   }

}
