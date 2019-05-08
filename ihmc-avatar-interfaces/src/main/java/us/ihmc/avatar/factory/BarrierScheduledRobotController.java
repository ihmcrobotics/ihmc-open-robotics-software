package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class BarrierScheduledRobotController<C> implements DisposableRobotController
{
   private final YoVariableRegistry registry;
   private final BarrierScheduler<C> barrierScheduler;
   private final YoLong schedulerTick;

   public BarrierScheduledRobotController(String name, List<? extends Task<C>> tasks, C masterContext, TaskOverrunBehavior overrunBehavior)
   {
      // TODO: add some YoVariables back that measure missed ticks.
      barrierScheduler = new BarrierScheduler<>(tasks, masterContext, overrunBehavior);
      registry = new YoVariableRegistry(name);
      schedulerTick = new YoLong("SchedulerTick", registry);

      // Start the task threads.
      for (int i = 0; i < tasks.size(); i++)
      {
         new Thread(tasks.get(i), "TaskThread" + i).start();
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      schedulerTick.increment();
      barrierScheduler.run();
   }

   @Override
   public void dispose()
   {
      barrierScheduler.shutdown();
   }

}
