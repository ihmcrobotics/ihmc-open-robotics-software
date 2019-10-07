package us.ihmc.quadrupedCommunication.networkProcessing;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ScheduledFuture;

public abstract class QuadrupedToolboxController
{
   protected static final boolean DEBUG = false;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final OutputManager outputManager;
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   protected final QuadrupedRobotDataReceiver robotDataReceiver;

   private ScheduledFuture<?> futureToListenTo;

   public QuadrupedToolboxController(QuadrupedRobotDataReceiver robotDataReceiver, OutputManager outputManager, YoVariableRegistry parentRegistry)
   {
      this.robotDataReceiver = robotDataReceiver;
      this.outputManager = outputManager;
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
    * Internal initialization method for preparing this toolbox controller before running
    * {@link #updateInternal()}.
    *
    * @return {@code true} if the initialization succeeded, {@code false} otherwise.
    */
   public boolean initialize() throws InterruptedException
   {
      if (robotDataReceiver != null)
      {
         while (!robotDataReceiver.framesHaveBeenSetUp())
         {
            robotDataReceiver.updateRobotModel();
            Thread.sleep(10);
         }
      }

      return initializeInternal();
   }

   /**
    * Initializes once and run the {@link #updateInternal()} of this toolbox controller only if the
    * initialization succeeded.
    * @throws Exception
    *
    * @see #hasBeenInitialized() The initialization state of this toolbox controller.
    */
   public void update() throws InterruptedException
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

      if (futureToListenTo != null)
      {
         if (futureToListenTo.isDone())
         {
            try
            {
               futureToListenTo.get();
            }
            catch (ExecutionException e)
            {
               e.getCause().printStackTrace();
            }
            catch (InterruptedException exception)
            {
               exception.getCause().printStackTrace();
            }

            throw new RuntimeException("Toolbox controller thread crashed.");
         }
      }
   }

   public void setFutureToListenTo(ScheduledFuture<?> future)
   {
      futureToListenTo = future;
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
    * @param outputMessage the message to publish.
    */
   public <S extends Settable<S>> void reportMessage(S outputMessage)
   {
      if (outputMessage != null)
         outputManager.reportMessage(outputMessage);
   }



   /**
    * Internal initialization method for preparing this toolbox controller before running
    * {@link #updateInternal()}.
    *
    * @return {@code true} if the initialization succeeded, {@code false} otherwise.
    */
   abstract public boolean initializeInternal();

   /**
    * Internal update method that should perform the computation for this toolbox controller. It is
    * called only if {@link #initialize()} has succeeded.
    *
    * @throws Exception
    */
   abstract public void updateInternal() throws Exception;

   abstract public boolean isDone();
}
