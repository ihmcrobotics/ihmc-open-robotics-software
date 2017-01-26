package us.ihmc.robotics.stateMachines.eventBasedStateMachine;

public interface FiniteStateMachineState<E extends Enum<E>>
{
   /**
    * Called when the state machine is transitioning into this state.
    */
   void onEntry();

   /**
    * Called regularly by the state machine to allow the state to do any time-dependent internal processing.
    *
    * @return a new event to be fired in the state machine, or <code>null</code> if no event is to be fired
    */
   E process();

   /**
    * Called when the state machine is transitioning out of this state.
    */
   void onExit();
}
