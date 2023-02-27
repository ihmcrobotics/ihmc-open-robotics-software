package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BarrierScheduledRobotController implements DisposableRobotController
{
   private final YoRegistry registry;
   private final BarrierScheduler<HumanoidRobotContextData> barrierScheduler;
   private final HumanoidRobotContextData masterContext;

   private final ThreadTimer timer;
   private final List<HumanoidRobotControlTask> tasks;

   public BarrierScheduledRobotController(String name,
                                          List<HumanoidRobotControlTask> tasks,
                                          HumanoidRobotContextData masterContext,
                                          TaskOverrunBehavior overrunBehavior,
                                          double schedulerDt)
   {
      this.tasks = tasks;
      this.masterContext = masterContext;

      barrierScheduler = new BarrierScheduler<>(tasks, masterContext, overrunBehavior);
      registry = new YoRegistry(name);

      timer = new ThreadTimer("Scheduler", schedulerDt, registry);
   }

   @Override
   public void initialize()
   {
      // Reset the internal counter to observe when the tasks get triggered.
      timer.reset();
      // Reset the scheduler counter so the threads restart as if we started for the first time.
      barrierScheduler.reset();
      for (int i = 0; i < tasks.size(); i++)
      {
         tasks.get(i).initialize();
      }
      masterContext.setControllerRan(false);
      masterContext.setEstimatorRan(false);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      timer.start();
      masterContext.setSchedulerTick(timer.getTickCount());
      barrierScheduler.run();
      timer.stop();
   }

   public void waitUntilTasksDone()
   {
      try
      {
         barrierScheduler.waitUntilTasksDone();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void dispose()
   {
      barrierScheduler.shutdown();
   }

}
