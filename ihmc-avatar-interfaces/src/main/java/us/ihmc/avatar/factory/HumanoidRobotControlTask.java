package us.ihmc.avatar.factory;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;

// TODO: move timing and registry updates to this abstract class.
public abstract class HumanoidRobotControlTask extends Task<HumanoidRobotContextData>
{
   public HumanoidRobotControlTask(long divisor)
   {
      super(divisor);
   }
}
