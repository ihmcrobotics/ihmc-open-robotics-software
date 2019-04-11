package us.ihmc.humanoidBehaviors.tools.state;

import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;

/**
 * This factory helps by adding complete do nothing state implementations for every enum value
 * and providing a one to many state transition condition.
 */
public class EnhancedStateMachineFactory<K extends Enum<K>>
{
   private final Class<K> keyType;
   private final StateMachineFactory<K, State> factory;
   private final HashMap<K, FriendlyState> stateMap;

   /**
    * Create default name and registry. Initialize with all enum values as a FriendlyState.
    *
    * @param keyType
    */
   public EnhancedStateMachineFactory(Class<K> keyType)
   {
      factory = new StateMachineFactory<>(keyType);
      this.keyType = keyType;
      String name = keyType.getSimpleName() + "Machine";
      getFactory().setNamePrefix(name).setRegistry(new YoVariableRegistry(name + "Registry"));

      stateMap = new HashMap<>();
      for (K value : EnumSet.allOf(keyType))
      {
         FriendlyState friendlyState = new FriendlyState();
         getFactory().addState(value, friendlyState);
         getStateMap().put(value, friendlyState);
      }
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

            return transitionTo == value; // must use == for null safety
         });
      }
   }

   public StateMachineFactory<K, State> getFactory()
   {
      return factory;
   }

   public HashMap<K, FriendlyState> getStateMap()
   {
      return stateMap;
   }
}
