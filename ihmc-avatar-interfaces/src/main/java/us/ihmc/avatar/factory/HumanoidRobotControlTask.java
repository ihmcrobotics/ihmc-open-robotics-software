package us.ihmc.avatar.factory;

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
}
