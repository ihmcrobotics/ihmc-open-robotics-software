package us.ihmc.aware.state;

import java.util.ArrayList;
import java.util.List;

public class StateMachineBuilder<S extends StateMachineState<E>, E extends Enum<E>>
{
   private final List<StateMachineTransition<S, E>> transitions = new ArrayList<>();

   public StateMachineBuilder addTransition(E event, S from, S to)
   {
      StateMachineTransition<S, E> transition = new StateMachineTransition<>(event, from, to);
      transitions.add(transition);

      return this;
   }

   public StateMachine<S, E> build(S initialState)
   {
      return new StateMachine<>(transitions, initialState);
   }
}
