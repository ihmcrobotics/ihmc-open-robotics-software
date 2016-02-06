package us.ihmc.aware.state;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class StateMachineBuilder<S extends Enum<S>, E extends Enum<E>>
{
   private final Class<S> enumType;
   private final String yoVariableName;
   private final YoVariableRegistry registry;

   private final Map<S, StateMachineState<E>> states = new HashMap<>();
   private final List<StateMachineTransition<S, E>> transitions = new ArrayList<>();

   public StateMachineBuilder(Class<S> enumType, String yoVariableName, YoVariableRegistry registry)
   {
      this.enumType = enumType;
      this.yoVariableName = yoVariableName;
      this.registry = registry;
   }

   public StateMachineBuilder addState(S state, StateMachineState<E> instance)
   {
      states.put(state, instance);

      return this;
   }

   public StateMachineBuilder addTransition(E event, S from, S to)
   {
      StateMachineTransition<S, E> transition = new StateMachineTransition<>(event, from, to);
      transitions.add(transition);

      return this;
   }

   public StateMachine<S, E> build(S initialState)
   {
      return new StateMachine<>(states, transitions, initialState, enumType, yoVariableName, registry);
   }
}
