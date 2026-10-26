package us.ihmc.avatar.factory;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.SingleThreadedScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class SingleThreadedRobotController<C> implements DisposableRobotController
{
   private final YoRegistry registry;
   private SingleThreadedScheduler<C> singleThreadedScheduler;
   private final YoLong schedulerTick;
   private final List<? extends Task<C>> tasks;
   private final C masterContext;

   public SingleThreadedRobotController(String name, List<? extends Task<C>> tasks, C masterContext)
   {
      this.tasks = new ArrayList<>(tasks);
      this.masterContext = masterContext;
      singleThreadedScheduler = new SingleThreadedScheduler<>(tasks, masterContext);
      registry = new YoRegistry(name);
      schedulerTick = new YoLong("SchedulerTick", registry);
   }

   @Override
   public void initialize()
   {
      // Re-initialize the tasks and restart the scheduler from its first tick, so everything runs at
      // the same phase as when we started for the first time, mirroring
      // BarrierScheduledRobotController.initialize().
      singleThreadedScheduler = new SingleThreadedScheduler<>(tasks, masterContext);
      schedulerTick.set(0);

      for (int i = 0; i < tasks.size(); i++)
      {
         if (tasks.get(i) instanceof HumanoidRobotControlTask humanoidRobotControlTask)
            humanoidRobotControlTask.initialize();
      }

      if (masterContext instanceof HumanoidRobotContextData humanoidRobotContextData)
      {
         humanoidRobotContextData.setControllerRan(false);
         humanoidRobotContextData.setEstimatorRan(false);
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      schedulerTick.increment();
      singleThreadedScheduler.run();
   }

   @Override
   public void dispose()
   {
      singleThreadedScheduler.shutdown();
   }

}
