package us.ihmc.tools.thread;

/**
 * A tool to provide a boolean enablement for a thread such as in graphics
 * engines that must poll every tick and need to check a boolean value every
 * tick. It also provides access to whether the value is newly changed via
 * #activationChanged. This is similar to Notification but does not clear if
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

   public boolean poll()
   {
      activationChanged = activated != previousValue;
      previousValue = activated;
      return activated;
   }

   public boolean read()
   {
      return previousValue;
   }

   public boolean activationChanged()
   {
      return activationChanged;
   }

   public void activate()
   {
      activated = true;
   }

   public void deactivate()
   {
      activated = false;
   }
}
