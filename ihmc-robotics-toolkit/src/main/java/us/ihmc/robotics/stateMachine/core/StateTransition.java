package us.ihmc.robotics.stateMachine.core;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * This class gathers the information necessary to encode any type of state transitions to be used
 * in a {@link StateMachine}.
 * <p>
 * A {@link StateTransition} is associated with a source {@link State} and holds onto a collection
 * of conditions each one associated with a state target. When the source state is active and a
 * condition is fulfilled, i.e. {@code StateTransitionCondition#testCondition(double)} returns
 * {@code true}, the state machine will perform a transition from the source state to the target
 * state associated with the condition.
 * </p>
 * <p>
 * This class is iterable over the registered target state keys.
 * </p>
 * 
 * @author Sylvain
 *
 * @param <K> Type of {@link Enum} that lists the potential states and that is used by the state
 *           machine this listener is to be added to.
 */
public final class StateTransition<K extends Enum<K>> implements Iterable<K>
{
   private final List<K> toStateKeys = new ArrayList<>();
   private final Map<K, List<StateTransitionCondition>> allConditions;

   /**
    * Creates an empty state transition.
    * <p>
    * When a state has an empty state transition, it cannot be exited.
    * </p>
    * 
    * @param keyType the type of the key to use, it is only used to create an {@code EnumMap} which
    *           requires it.
    */
   public StateTransition(Class<K> keyType)
   {
      allConditions = new EnumMap<>(keyType);
   }

   /**
    * Creates a state transition and initializes it with a target state and a condition.
    * 
    * @param to key of the target state.
    * @param condition the condition that when fulfilled, the state machine will trigger a transition
    *           to the target state.
    */
   public StateTransition(K to, StateTransitionCondition condition)
   {
      allConditions = new EnumMap<>(to.getDeclaringClass());
      addCondition(to, condition);
   }

   /**
    * Extracts the conditions and target states from {@code other} and adds them to this state
    * transition.
    * 
    * @param other the other state transition used to extend this.
    */
   public void completeWith(StateTransition<K> other)
   {
      other.allConditions.entrySet().forEach(e -> addConditions(e.getKey(), e.getValue()));
   }

   /**
    * Adds multiple conditions for reaching a single target state.
    * <p>
    * It is enough to have a single condition out of the given ones to be fulfilled to trigger a
    * transition to the target state.
    * </p>
    * 
    * @param to key of the target state.
    * @param conditions the different conditions to reach the target state.
    */
   public void addConditions(K to, Iterable<? extends StateTransitionCondition> conditions)
   {
      conditions.forEach(condition -> addCondition(to, condition));
   }

   /**
    * Add a new condition for reaching a target state.
    * <p>
    * If conditions were already added for the same target state, only one of them has to be fulfilled
    * to trigger a transition.
    * </p>
    * 
    * @param to key of the target state.
    * @param condition the new condition to reach the target state.
    */
   public void addCondition(K to, StateTransitionCondition condition)
   {
      if (!allConditions.containsKey(to))
      {
         toStateKeys.add(to);
         allConditions.put(to, new ArrayList<>());
      }

      allConditions.get(to).add(condition);
   }

   /**
    * Invoked by the state machine to identify when to trigger a transition and to which state.
    * 
    * @param timeInCurrentState the time spent in the current state or {@link Double#NaN} if the time
    *           information is unavailable.
    * @return {@code null} is no transition is requested, or the key of the state to transition to.
    */
   K isTransitionRequested(double timeInCurrentState)
   {
      for (int i = 0; i < toStateKeys.size(); i++)
      {
         K to = toStateKeys.get(i);
       
         List<StateTransitionCondition> toStateConditions = allConditions.get(to);

         for (int j = 0; j < toStateConditions.size(); j++)
         {
            if (toStateConditions.get(j).testCondition(timeInCurrentState))
               return to;
         }
      }

      return null;
   }

   /**
    * The number of target states for which at least one condition has been registered.
    * <p>
    * Assuming all the conditions are fulfillable, each of the target state can be transitioned into
    * from the source state to which this state transition is associated.
    * </p>
    * 
    * @return the number of target states registered in this state transition.
    */
   public int getNumberOfTargetStates()
   {
      return toStateKeys.size();
   }

   /**
    * Creates an iterator that can be used to iterate through the keys of all the target states
    * registered.
    */
   @Override
   public Iterator<K> iterator()
   {
      return toStateKeys.iterator();
   }
}
