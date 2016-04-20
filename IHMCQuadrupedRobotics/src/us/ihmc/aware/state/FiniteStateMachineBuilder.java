package us.ihmc.aware.state;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class FiniteStateMachineBuilder<S extends Enum<S>, E extends Enum<E>>
{
   private final Class<S> enumType;
   private final String yoVariableName;
   private final YoVariableRegistry registry;

   private final Map<S, FiniteStateMachineState<E>> states = new HashMap<>();
   private final List<FiniteStateMachineTransition<S, E>> transitions = new ArrayList<>();

   public FiniteStateMachineBuilder(Class<S> enumType, String yoVariableName, YoVariableRegistry registry)
   {
      this.enumType = enumType;
      this.yoVariableName = yoVariableName;
      this.registry = registry;
   }

   public FiniteStateMachineBuilder addState(S state, FiniteStateMachineState<E> instance)
   {
      states.put(state, instance);

      return this;
   }

   public FiniteStateMachineBuilder addTransition(E event, S from, S to)
   {
      FiniteStateMachineTransition<S, E> transition = new FiniteStateMachineTransition<>(event, from, to);
      transitions.add(transition);

      return this;
   }

   public FiniteStateMachine<S, E> build(S initialState)
   {
      return new FiniteStateMachine<>(states, transitions, initialState, enumType, yoVariableName, registry);
   }
}
