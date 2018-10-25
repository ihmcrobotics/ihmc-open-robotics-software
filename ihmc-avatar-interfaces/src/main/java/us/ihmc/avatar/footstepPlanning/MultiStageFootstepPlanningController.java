package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.YoFootstepPlannerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessageConverter;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class MultiStageFootstepPlanningController
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final MultiStageFootstepPlanningManager stageManager;

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean requestedPlanarRegions = new YoBoolean("RequestedPlanarRegions", registry);
   private final YoDouble toolboxTime = new YoDouble("ToolboxTime", registry);
   private final YoDouble timeout = new YoDouble("ToolboxTimeout", registry);
   private final YoInteger planId = new YoInteger("planId", registry);

   private Runnable managerRunnable = null;
   private ScheduledFuture<?> managerTask = null;

   private double dt;
   private final ScheduledExecutorService executorService;
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   public MultiStageFootstepPlanningController(RobotContactPointParameters<RobotSide> contactPointParameters,
                                               FootstepPlannerParameters footstepPlannerParameters, StatusMessageOutputManager statusOutputManager,
                                               ScheduledExecutorService executorService, YoVariableRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry, double dt)
   {
      this.dt = dt;
      this.executorService = executorService;
      stageManager = new MultiStageFootstepPlanningManager(contactPointParameters, footstepPlannerParameters, statusOutputManager, executorService, parentRegistry, graphicsListRegistry, dt);

      isDone.set(true);
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);

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


   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> publisher)
   {
      stageManager.setTextToSpeechPublisher(publisher);
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
      managerTask = executorService.scheduleAtFixedRate(managerRunnable, 0, (long) Conversions.secondsToMilliseconds(dt), TimeUnit.MILLISECONDS);

      stageManager.wakeUp();
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
               toolboxTime.add(dt);

               //               if (receivedInput.getAndSet(false))
               //                  timeOfLastInput.set(yoTime.getDoubleValue());
               //               if (yoTime.getDoubleValue() - timeOfLastInput.getDoubleValue() >= timeWithoutInputsBeforeGoingToSleep.getDoubleValue())
               //                  sleep();
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
