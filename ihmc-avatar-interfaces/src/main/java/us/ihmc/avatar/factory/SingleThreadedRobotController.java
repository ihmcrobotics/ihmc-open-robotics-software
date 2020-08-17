package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.SingleThreadedScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class SingleThreadedRobotController<C> implements DisposableRobotController
{
   private final YoRegistry registry;
   private final SingleThreadedScheduler<C> singleThreadedScheduler;
   private final YoLong schedulerTick;

   public SingleThreadedRobotController(String name, List<? extends Task<C>> tasks, C masterContext)
   {
      singleThreadedScheduler = new SingleThreadedScheduler<>(tasks, masterContext);
      registry = new YoRegistry(name);
      schedulerTick = new YoLong("SchedulerTick", registry);
   }

   @Override
   public void initialize()
   {
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
