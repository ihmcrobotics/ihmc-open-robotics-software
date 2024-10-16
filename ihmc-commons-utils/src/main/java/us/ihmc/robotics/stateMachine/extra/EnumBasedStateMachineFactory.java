package us.ihmc.robotics.stateMachine.extra;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoublePredicate;
import java.util.function.Supplier;

import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This factory helps by adding complete do nothing state implementations for every enum value
 * and providing a one to many state transition condition.
 */
public class EnumBasedStateMachineFactory<K extends Enum<K>>
{
   private final Class<K> keyType;
   private final StateMachineFactory<K, State> factory;
   private final HashMap<K, MutableState> stateMap;

   /**
    * Create default name and registry. Initialize with all enum values as a MutableState.
    *
    * @param keyType
    */
   public EnumBasedStateMachineFactory(Class<K> keyType)
   {
      factory = new StateMachineFactory<>(keyType);
      this.keyType = keyType;
      String name = keyType.getSimpleName() + "Machine";
      getFactory().setNamePrefix(name).setRegistry(new YoRegistry(name + "Registry"));

      stateMap = new HashMap<>();
      for (K value : EnumSet.allOf(keyType))
      {
         MutableState mutableState = new MutableState();
         getFactory().addState(value, mutableState);
         getStateMap().put(value, mutableState);
      }
   }

   public void addTransition(K from, K to, StateTransitionCondition condition)
   {
      getFactory().addTransition(from,  to, condition);
   }

   public void addTransition(K from, K to, BooleanSupplier condition)
   {
      getFactory().addTransition(from,  to, timeInCurrentState -> condition.getAsBoolean());
   }

   public void addTransition(K from, List<K> toOptions, Supplier<K> stateTransitionTo)
   {
      addTransition(from, toOptions, timeInCurrentState -> stateTransitionTo.get());
   }

   /**
    * Allows to have a single condition which returns the state to switch to or null.
    *
    * Note: Requires that application state is not changed in the condition, as it will be called many times per tick.
    */
   public void addTransition(K from, List<K> toOptions, StateTransitionTo<K> stateTransitionTo)
   {
      for (K value : toOptions)
      {
         factory.addTransition(from, value, timeInState ->
         {
            K transitionTo = stateTransitionTo.shouldTransitionTo(timeInState);

            if (transitionTo != null && !toOptions.contains(transitionTo)) // must check null here
            {
               throw new RuntimeException("Invalid transition to " + transitionTo + ". Options are " + toOptions);
            }

            if (transitionTo == value)
            {
               LogTools.debug("Transition {} -> {}", from.name(), transitionTo.name());
            }
            
            return transitionTo == value; // must use == for null safety
         });
      }
   }

   public StateMachineFactory<K, State> getFactory()
   {
      return factory;
   }

   public HashMap<K, MutableState> getStateMap()
   {
      return stateMap;
   }

   public MutableState getState(K stateEnum)
   {
      return stateMap.get(stateEnum);
   }

   public void setOnEntry(K key, Runnable onEntry)
   {
      getState(key).setOnEntry(onEntry);
   }

   public void setDoAction(K key, Runnable doAction)
   {
      setDoAction(key, timeInCurrentState -> doAction.run());
   }

   public void setDoAction(K key, DoubleConsumer doAction)
   {
      getState(key).setDoAction(doAction);
   }

   public void setOnExit(K key, Runnable onExit)
   {
      getState(key).setOnExit(onExit);
   }

   public void setIsDone(K key, DoublePredicate isDone)
   {
      getState(key).setIsDone(isDone);
   }
}