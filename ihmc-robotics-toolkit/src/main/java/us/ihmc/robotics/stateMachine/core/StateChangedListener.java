package us.ihmc.robotics.stateMachine.core;

/**
 * Listener interface to use with {@link StateMachine} for getting notified of state changes.
 * 
 * 
 * @author Sylvain
 *
 * @param <K> Type of {@link Enum} that lists the potential states and that is used by the state
 *           machine this listener is to be added to.
 */
public interface StateChangedListener<K>
{
   /**
    * Invoked when the active state is changing.
    * 
    * @param from key of the previous active state.
    * @param to key of the new active state.
    */
   void stateChanged(K from, K to);
}
