package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BarrierScheduledRobotController implements DisposableRobotController
{
   private final YoVariableRegistry registry;
   private final BarrierScheduler<HumanoidRobotContextData> barrierScheduler;
   private final HumanoidRobotContextData masterContext;

   private final ThreadTimer timer;

   public BarrierScheduledRobotController(String name, List<HumanoidRobotControlTask> tasks, HumanoidRobotContextData masterContext,
                                          TaskOverrunBehavior overrunBehavior, double schedulerDt)
   {
      this.masterContext = masterContext;

      barrierScheduler = new BarrierScheduler<>(tasks, masterContext, overrunBehavior);
      registry = new YoVariableRegistry(name);

      timer = new ThreadTimer("Scheduler", schedulerDt, registry);
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
      masterContext.setSchedulerTick(timer.getTickCount());
      barrierScheduler.run();
      timer.stop();
   }

   @Override
   public void dispose()
   {
      barrierScheduler.shutdown();
   }

}
