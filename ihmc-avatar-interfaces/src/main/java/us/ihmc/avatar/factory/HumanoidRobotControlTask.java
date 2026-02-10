package us.ihmc.avatar.factory;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;

public abstract class HumanoidRobotControlTask extends Task<HumanoidRobotContextData>
{
   protected final List<Runnable> preTaskCallbacks = new ArrayList<>();
   protected final List<Runnable> postTaskCallbacks = new ArrayList<>();
   protected final List<Runnable> startupRunnables = new ArrayList<>();
   protected final List<Runnable> cleanupRunnables = new ArrayList<>();
   protected final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   public HumanoidRobotControlTask(long divisor)
   {
      super(divisor);
   }

   @Override
   protected boolean initialize()
   {
      runAll(startupRunnables);

      return super.initialize();
   }

   @Override
   protected void cleanup()
   {
      runAll(cleanupRunnables);
   }

   /**
    * The given callback will be run on the task thread right <b>before</b> each control tick.
    */
   public void addCallbackPreTask(Runnable callback)
   {
      preTaskCallbacks.add(callback);
   }

   /**
    * The given callback will be run on the task thread right <b>after</b> each control tick.
    */
   public void addCallbackPostTask(Runnable callback)
   {
      postTaskCallbacks.add(callback);
   }

   /**
    * This will cause the provided runnable to be executed on the scheduler thread before the task is
    * released. All runnables provided here will be executed periodically in the order they were
    * provided.
    *
    * @param runnable
    */
   public void addRunnableOnSchedulerThread(Runnable runnable)
   {
      schedulerThreadRunnables.add(runnable);
   }

   /**
    * This will cause the provided runnable to be executed on initialize().
    *
    * @param runnable
    */
   public void addRunnableOnStartup(Runnable runnable)
   {
      startupRunnables.add(runnable);
   }

   /**
    * This will cause the provided runnable to be executed on cleanup().
    *
    * @param runnable
    */
   public void addRunnableOnCleanup(Runnable runnable)
   {
      cleanupRunnables.add(runnable);
   }

   protected static void runAll(List<Runnable> runnables)
   {
      for (int i = 0; i < runnables.size(); i++)
         runnables.get(i).run();
   }
}
