package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;

public abstract class HumanoidRobotControlTask extends Task<HumanoidRobotContextData>
{
   public HumanoidRobotControlTask(long divisor)
   {
      super(divisor);
   }

   @Override
   protected boolean initialize()
   {
      return true;
   }

   @Override
   protected void cleanup()
   {
   }

   /**
    * The given callback will be run on the task thread right <b>before</b> each control tick.
    */
   public void addCallbackPreTask(Runnable callback)
   {
      throw new UnsupportedOperationException(getClass().getSimpleName() + " does not support this operation.");
   }

   /**
    * The given callback will be run on the task thread right <b>after</b> each control tick.
    */
   public void addCallbackPostTask(Runnable callback)
   {
      throw new UnsupportedOperationException(getClass().getSimpleName() + " does not support this operation.");
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
      throw new UnsupportedOperationException(getClass().getSimpleName() + " does not support this operation.");
   }

   protected static void runAll(List<Runnable> runnables)
   {
      for (int i = 0; i < runnables.size(); i++)
         runnables.get(i).run();
   }
}
