package us.ihmc.aware.state;

public interface StateMachineState<E extends Enum<E>>
{
   /**
    * Called when the state machine is transitioning into this state.
    */
   void onEntry();

   E process();

   /**
    * Called when the state machine is transitioning out of this state.
    */
   void onExit();
}
