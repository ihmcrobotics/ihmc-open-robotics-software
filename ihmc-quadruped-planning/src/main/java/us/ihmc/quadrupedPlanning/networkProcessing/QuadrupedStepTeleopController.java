package us.ihmc.quadrupedPlanning.networkProcessing;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.input.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.input.QuadrupedStepTeleopManager;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class QuadrupedStepTeleopController
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StatusMessageOutputManager statusOutputManager;
   private final QuadrupedStepTeleopManager teleopManager;

   private final YoDouble timeWithoutInputsBeforeGoingToSleep = new YoDouble("timeWithoutInputsBeforeGoingToSleep", registry);

   private final YoDouble controllerTime = new YoDouble("TeleopControllerTime", registry);
   protected final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);

   private Runnable managerRunnable = null;
   private ScheduledFuture<?> managerTask = null;

   protected final AtomicBoolean receivedInput = new AtomicBoolean();

   private final long tickTimeMs;
   private final ScheduledExecutorService executorService;
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   public QuadrupedStepTeleopController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialBodyHeight, CommandInputManager commandInputManager,
                                        StatusMessageOutputManager statusOutputManager, QuadrupedRobotModelProviderNode robotModelProvider,
                                        ScheduledExecutorService executorService, YoVariableRegistry parentRegistry,
                                        YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      this.tickTimeMs = tickTimeMs;
      this.executorService = executorService;
      this.statusOutputManager = statusOutputManager;
      teleopManager = new QuadrupedStepTeleopManager(defaultXGaitSettings, initialBodyHeight, robotModelProvider.getReferenceFrames(),
                                                     Conversions.millisecondsToSeconds(tickTimeMs), graphicsListRegistry, registry);

      commandInputManager.registerHasReceivedInputListener(command -> receivedInput.set(true));

      timeWithoutInputsBeforeGoingToSleep.set(Double.POSITIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public void requestInitialize()
   {
      initialize.set(true);
   }

   public void setPaused(boolean pause)
   {
      teleopManager.setPaused(pause);
   }

   public void processBodyPathPlanMessage(QuadrupedBodyPathPlanMessage message)
   {
      teleopManager.processBodyPathPlanMessage(message);
   }

   public void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      teleopManager.processFootstepStatusMessage(message);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      teleopManager.processHighLevelStateChangeMessage(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      teleopManager.processSteppingStateChangeMessage(message);
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      teleopManager.processGroundPlaneMessage(message);
   }

   public void processTimestamp(long timestampInNanos)
   {
      teleopManager.processTimestamp(timestampInNanos);
   }

   public void reinitialize()
   {
      requestInitialize();
   }

   public void wakeUp()
   {
      if (managerTask != null)
      {
         if (debug)
            PrintTools.error(this, "This planner is already running.");
         return;
      }

      if (debug)
         PrintTools.debug(this, "Waking up");

      createPlannerRunnable();
      teleopManager.wakeUp();
      managerTask = executorService.scheduleAtFixedRate(managerRunnable, 0, tickTimeMs, TimeUnit.MILLISECONDS);

      receivedInput.set(true);
   }

   public void sleep()
   {
      if (debug)
         PrintTools.debug(this, "Going to sleep");

      teleopManager.sleep();

      managerRunnable = null;

      if (managerTask == null)
      {
         if (debug)
            PrintTools.error(this, "There is no task running.");
         return;
      }
      else
      {
         managerTask.cancel(true);
         managerTask = null;
      }
   }

   public void destroy()
   {
      sleep();

      if (debug)
         PrintTools.debug(this, "Destroyed");
   }

   private void createPlannerRunnable()
   {
      if (managerRunnable != null)
      {
         if (debug)
            PrintTools.error(this, "This planning runnable is not null.");
         return;
      }

      managerRunnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            try
            {
               teleopManager.update();
               statusOutputManager.reportStatusMessage(teleopManager.getStepListMessage());
               statusOutputManager.reportStatusMessage(teleopManager.getBodyOrientationMessage());
               statusOutputManager.reportStatusMessage(teleopManager.getBodyHeightMessage());

               controllerTime.add(Conversions.millisecondsToSeconds(tickTimeMs));

               if (receivedInput.getAndSet(false))
                  timeOfLastInput.set(controllerTime.getDoubleValue());
               if (controllerTime.getDoubleValue() - timeOfLastInput.getDoubleValue() >= timeWithoutInputsBeforeGoingToSleep.getDoubleValue())
                  sleep();
            }
            catch (Exception e)
            {
               e.printStackTrace();
               sleep();
               throw e;
            }
         }
      };
   }
}
