package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.SingleThreadedScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class SingleThreadedRobotController<C> implements DisposableRobotController
{
   private final YoVariableRegistry registry;
   private final SingleThreadedScheduler<C> singleThreadedScheduler;
   private final YoLong schedulerTick;

   public SingleThreadedRobotController(String name, List<? extends Task<C>> tasks, C masterContext)
   {
      singleThreadedScheduler = new SingleThreadedScheduler<>(tasks, masterContext);
      registry = new YoVariableRegistry(name);
      schedulerTick = new YoLong("SchedulerTick", registry);
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
      singleThreadedScheduler.run();
   }

   @Override
   public void dispose()
   {
      singleThreadedScheduler.shutdown();
   }

}
