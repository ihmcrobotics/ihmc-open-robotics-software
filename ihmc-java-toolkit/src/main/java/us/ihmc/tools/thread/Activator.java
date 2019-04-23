package us.ihmc.tools.thread;

/**
 * A tool to provide a boolean enablement for a thread such as in graphics
 * engines that must poll every tick and need to check a boolean value every
 * tick. It also provides access to whether the value is newly changed via
 * {@link #hasChanged()}. This is similar to Notification but does not clear if
 * it is polled.
 * <p/>
 * The Activation/Activator classes are to set from another thread an object
 * which has a persistent value, for instance "activating" some functionality.
 * It is handy also that it provides access to if the value/activated-ness
 * just changed, to assist when state transition action is needed.
 */
public class Activator
{
   private boolean activated = false;
   private boolean previousValue = false;
   private boolean activationChanged = false;

   /**
    * Access the volatile boolean, store it and whether it changed for later
    * calls to {@link #peek()} and {@link #hasChanged()}.
    *
    * @return activated
    */
   public synchronized boolean poll()
   {
      activationChanged = activated != previousValue;
      previousValue = activated;
      return activated;
   }

   /**
    * Must have called {@link #poll()} first! Get activation state.
    *
    * @return activated
    */
   public boolean peek()
   {
      return previousValue;
   }

   /**
    * Must have called {@link #poll()} first! Get whether the activation state changed on
    * the last poll.
    *
    * @return activation changed
    */
   public boolean hasChanged()
   {
      return activationChanged;
   }

   /** THREAD 2 ACCESS BELOW THIS POINT */

   /**
    * Set activated. Called from external thread.
    */
   public synchronized void activate()
   {
      activated = true;
   }

   /**
    * Set deactivated. Called from external thread.
    */
   public synchronized void deactivate()
   {
      activated = false;
   }
}
