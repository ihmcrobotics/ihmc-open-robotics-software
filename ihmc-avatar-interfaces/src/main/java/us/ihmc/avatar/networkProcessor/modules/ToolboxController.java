package us.ihmc.avatar.networkProcessor.modules;

import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class ToolboxController
{
   private static final boolean DEBUG = false;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final StatusMessageOutputManager statusOutputManager;
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   public ToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;
      parentRegistry.addChild(registry);
      requestInitialize();
   }

   /**
    * Request this toolbox controller to run {@link #initialize} on the next call of
    * {@link #update()}.
    */
   public void requestInitialize()
   {
      initialize.set(true);
   }

   /**
    * Initializes once and run the {@link #updateInternal()} of this toolbox controller only if the
    * initialization succeeded.
    * 
    * @see #hasBeenInitialized() The initialization state of this toolbox controller.
    */
   public void update()
   {
      if (initialize.getBooleanValue())
      {
         if (!initialize()) // Return until the initialization succeeds
            return;
         initialize.set(false);
      }

      try
      {
         updateInternal();
      }
      catch (Exception e)
      {
         if (DEBUG)
         {
            e.printStackTrace();
         }
      }
   }

   /**
    * Get the initialization state of this toolbox controller:
    * <ul>
    * <li>{@code true}: this toolbox controller has been initialized properly and is ready for doing
    * some computation!
    * <li>{@code false}: this toolbox controller has either not been initialized yet or the
    * initialization process failed.
    * </ul>
    * 
    * @return the initialization state of this toolbox controller.
    */
   public boolean hasBeenInitialized()
   {
      return !initialize.getValue();
   }

   /**
    * Publishes the given status message. It used for sending the result computed by this toolbox
    * controller.
    * 
    * @param statusMessage the message to publish.
    */
   protected <S extends Settable<S>> void reportMessage(S statusMessage)
   {
      statusOutputManager.reportStatusMessage(statusMessage);
   }

   /**
    * Internal initialization method for preparing this toolbox controller before running
    * {@link #updateInternal()}.
    * 
    * @return {@code true} if the initialization succeeded, {@code false} otherwise.
    */
   abstract protected boolean initialize();

   /**
    * Internal update method that should perform the computation for this toolbox controller. It is
    * called only if {@link #initialize()} has succeeded.
    * 
    * @throws Exception
    */
   abstract protected void updateInternal() throws Exception;

   abstract protected boolean isDone();
}
