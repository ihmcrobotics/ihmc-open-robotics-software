package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.TreeMap;

public class StateChangeRecorder<E extends Enum<E>> implements StateChangedListener<E>
{
   private final LinkedHashMap<State<E>, ArrayList<Double>> statesAndTheirEntryTimes = new LinkedHashMap<State<E>, ArrayList<Double>>();
   private final LinkedHashSet<State<E>> encounteredStates = new LinkedHashSet<State<E>>();
   private final TreeMap<Double, State<E>> timeTable = new TreeMap<Double, State<E>>();

   public StateChangeRecorder()
   {
      // empty
   }

   public void stateChanged(State<E> oldState, State<E> newState, double time)
   {
      updateStatesAndTheirEntryTimes(newState, time);
      encounteredStates.add(newState);
      timeTable.put(time, newState);
   }

   public ArrayList<Double> getListOfSwitchTimes(State<E> stateToGetListFor)
   {
      ArrayList<Double> original = statesAndTheirEntryTimes.get(stateToGetListFor);

      ArrayList<Double> defensiveCopy = new ArrayList<Double>();

      if (original != null)
         defensiveCopy.addAll(original);

      return defensiveCopy;
   }

   public LinkedHashMap<State<E>, ArrayList<Double>> getStatesAndSwitchTimes()
   {
      return new LinkedHashMap<State<E>, ArrayList<Double>>(statesAndTheirEntryTimes);
   }

   public TreeMap<Double, State<E>> getTimeTable()
   {
      return new TreeMap<Double, State<E>>(timeTable);
   }

   public LinkedHashSet<State<E>> getEncounteredStates()
   {
      return new LinkedHashSet<State<E>>(encounteredStates);
   }

   public void clearAllData()
   {
      statesAndTheirEntryTimes.clear();
      encounteredStates.clear();
      timeTable.clear();
   }

   private void updateStatesAndTheirEntryTimes(State<E> newState, double time)
   {
      if (!statesAndTheirEntryTimes.containsKey(newState))
      {
         statesAndTheirEntryTimes.put(newState, new ArrayList<Double>());
      }

      ArrayList<Double> switchTimeList = statesAndTheirEntryTimes.get(newState);
      switchTimeList.add(time);
   }
}
