package us.ihmc.robotics.stateMachine.core;

import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;

/**
 * This interface gathers the backbone of a state that can be used to create a {@link StateMachine}.
 * <p>
 * {@link State} was designed to be a as simple as possible, i.e. not generic parameter and only a
 * handful of methods, and there is no reason for it to become more complicated.
 * </p>
 * 
 * @author Sylvain
 */
public interface State
{
   /**
    * Called when the state machine is transitioning into this state.
    */
   void onEntry();

   /**
    * Called regularly by the state machine to allow the state to do any time-dependent internal
    * processing.
    * 
    * @param timeInState represents the time spent in this state, or {@link Double#NaN} if the clock of
    *           the state machine was not setup.
    */
   void doAction(double timeInState);

   /**
    * Called when the state machine is transitioning out of this state.
    * 
    * @param timeInState represents the time spent in this state, or {@link Double#NaN} if the clock of
    *           the state machine was not setup.
    */
   default void onExit(double timeInState)
   {
      onExit();
   }
   
   
   /**
    * Deprecated, kept for backwards compatibility.
    * 
    * Use onExit(timeInState) instead.
    */
   @Deprecated
   default void onExit()
   {
      
   }
   

   /**
    * Invoked frequently by a state machine to identify when the active state is done. It will then
    * transition to the target state as defined when registering the state in
    * {@link StateMachineFactory}.
    * <p>
    * This method be overridden to be able to create the following transition:
    * {@link StateMachineFactory#addDoneTransition(Enum, Enum)}. It is only when a done-transition is
    * created that the state machine will call {@link State#isDone(double)}.
    * </p>
    * 
    * @param timeInState the time spent in this state.
    * @return {@code true} if this state is ready to be exited, {@code false} otherwise.
    */
   default boolean isDone(double timeInState)
   {
      return false;
   }
}
