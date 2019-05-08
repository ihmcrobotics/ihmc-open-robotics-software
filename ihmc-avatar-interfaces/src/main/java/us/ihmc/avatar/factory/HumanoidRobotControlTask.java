package us.ihmc.avatar.factory;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class HumanoidRobotControlTask extends Task<HumanoidRobotContextData>
{
   public HumanoidRobotControlTask(long divisor)
   {
      super(divisor);
   }

   public abstract YoVariableRegistry getRegistry();

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }
}
