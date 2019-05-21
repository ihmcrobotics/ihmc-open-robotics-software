package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BarrierScheduledRobotController<C> implements DisposableRobotController
{
   private final YoVariableRegistry registry;
   private final BarrierScheduler<C> barrierScheduler;

   private final ThreadTimer timer;

   public BarrierScheduledRobotController(String name, List<? extends Task<C>> tasks, C masterContext, TaskOverrunBehavior overrunBehavior)
   {
      // TODO: add some YoVariables back that measure missed ticks.
      barrierScheduler = new BarrierScheduler<>(tasks, masterContext, overrunBehavior);
      registry = new YoVariableRegistry(name);

      timer = new ThreadTimer("Scheduler", registry);
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
      timer.start();
      barrierScheduler.run();
      timer.stop();
   }

   @Override
   public void dispose()
   {
      barrierScheduler.shutdown();
   }

}
