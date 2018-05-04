package us.ihmc.robotics.stateMachine.old.eventBasedStateMachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

@Deprecated
public class FiniteStateMachineBuilder<S extends Enum<S>, E extends Enum<E>, C extends FiniteStateMachineState<E>>
{
   private final Class<S> enumType;
   private final Class<E> standardEventType;
   private final String yoVariableName;
   private final YoVariableRegistry registry;

   private final Map<S, C> states = new HashMap<>();
   private final Map<Class<?>, List<FiniteStateMachineTransition<S, ? extends Enum<?>>>> transitions = new HashMap<>();
   private final Map<Class<?>, List<FiniteStateMachineCallback<S, ? extends Enum<?>>>> callbacks = new HashMap<>();

   public FiniteStateMachineBuilder(Class<S> enumType, Class<E> standardEventType, String yoVariableName, YoVariableRegistry registry)
   {
      this.enumType = enumType;
      this.standardEventType = standardEventType;
      this.yoVariableName = yoVariableName;
      this.registry = registry;
   }

   public FiniteStateMachineBuilder addState(S stateEnum, C state)
   {
      states.put(stateEnum, state);

      return this;
   }

   public FiniteStateMachineBuilder addTransition(E event, S from, S to)
   {
      return addTransition(standardEventType, event, from, to);
   }

   public <M extends Enum<M>> FiniteStateMachineBuilder addTransition(Class<M> eventType, M event, S from, S to)
   {
      List<FiniteStateMachineTransition<S, ? extends Enum<?>>> transitionsOnE = transitions.get(eventType);
      if (transitionsOnE == null)
      {
         transitionsOnE = new ArrayList<>();
         transitions.put(eventType, transitionsOnE);
      }

      transitionsOnE.add(new FiniteStateMachineTransition<>(event, from, to));
      return this;
   }

   public FiniteStateMachineBuilder addCallback(E event, S state, Runnable callback)
   {
      return addCallback(standardEventType, event, state, callback);
   }

   public <M extends Enum<M>> FiniteStateMachineBuilder addCallback(Class<M> eventType, M event, S state, Runnable callback)
   {
      List<FiniteStateMachineCallback<S, ? extends Enum<?>>> callbacksOnE = callbacks.get(eventType);
      if (callbacksOnE == null)
      {
         callbacksOnE = new ArrayList<>();
         callbacks.put(eventType, callbacksOnE);
      }

      callbacksOnE.add(new FiniteStateMachineCallback<>(event, state, callback));
      return this;
   }

   public FiniteStateMachine<S, E, C> build(S initialState)
   {
      return new FiniteStateMachine<>(states, transitions, callbacks, initialState, enumType, standardEventType, yoVariableName, registry);
   }
}
