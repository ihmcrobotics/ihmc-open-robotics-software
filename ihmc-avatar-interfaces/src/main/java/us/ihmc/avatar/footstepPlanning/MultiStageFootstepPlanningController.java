package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class MultiStageFootstepPlanningController
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final MultiStageFootstepPlanningManager stageManager;

   private final YoDouble timeWithoutInputsBeforeGoingToSleep = new YoDouble("timeWithoutInputsBeforeGoingToSleep", registry);

   private final YoDouble controllerTime = new YoDouble("PlanningControllerTime", registry);
   protected final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);

   private Runnable managerRunnable = null;
   private ScheduledFuture<?> managerTask = null;

   protected final AtomicBoolean receivedInput = new AtomicBoolean();

   private final long tickTimeMs;
   private final ScheduledExecutorService executorService;
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   public MultiStageFootstepPlanningController(RobotContactPointParameters<RobotSide> contactPointParameters,
                                               FootstepPlannerParameters footstepPlannerParameters, CommandInputManager commandInputManager,
                                               StatusMessageOutputManager statusOutputManager, ScheduledExecutorService executorService,
                                               YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      this.tickTimeMs = tickTimeMs;
      this.executorService = executorService;
      stageManager = new MultiStageFootstepPlanningManager(contactPointParameters, footstepPlannerParameters, statusOutputManager, parentRegistry,
                                                           graphicsListRegistry, tickTimeMs);

      commandInputManager.registerHasReceivedInputListener(command -> receivedInput.set(true));

      timeWithoutInputsBeforeGoingToSleep.set(Double.POSITIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public void requestInitialize()
   {
      initialize.set(true);
   }

   public void processRequest(FootstepPlanningRequestPacket request)
   {
      stageManager.processRequest(request);
   }

   public void processPlannerParameters(FootstepPlannerParametersPacket parameters)
   {
      stageManager.processPlannerParameters(parameters);
   }

   public void processPlanningStatisticsRequest()
   {
      stageManager.processPlanningStatisticsRequest();
   }

   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher)
   {
      stageManager.setTextToSpeechPublisher(textToSpeechPublisher);
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
      managerTask = executorService.scheduleAtFixedRate(managerRunnable, 0, tickTimeMs, TimeUnit.MILLISECONDS);

      stageManager.wakeUp();
      receivedInput.set(true);
   }

   public void sleep()
   {
      if (debug)
         PrintTools.debug(this, "Going to sleep");

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
               stageManager.update();
               controllerTime.add(Conversions.millisecondsToSeconds(tickTimeMs));

               if (receivedInput.getAndSet(false))
                  timeOfLastInput.set(controllerTime.getDoubleValue());
               if (controllerTime.getDoubleValue() - timeOfLastInput.getDoubleValue() >= timeWithoutInputsBeforeGoingToSleep.getDoubleValue())
                  sleep();
               if (stageManager.isDone())
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
