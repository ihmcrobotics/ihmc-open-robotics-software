package us.ihmc.javaFXToolkit.messager;

/**
 * Implement this interface to create a listener that is to be notified when the state of a messager
 * is changing between open and closed.
 * 
 * @author Sylvain Bertrand
 */
public interface MessagerStateListener
{
   /**
    * The state of the messager just changed.
    * 
    * @param isMessagerOpen {@code true} is this messager is now open, i.e. ready to send and
    *           receive messages, {@code false} if it is now closed.
    */
   void messagerStateChanged(boolean isMessagerOpen);
}
