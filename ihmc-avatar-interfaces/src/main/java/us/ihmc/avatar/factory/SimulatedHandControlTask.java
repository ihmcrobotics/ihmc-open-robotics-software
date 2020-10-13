package us.ihmc.avatar.factory;

import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class SimulatedHandControlTask extends HumanoidRobotControlTask
{
   public SimulatedHandControlTask(long divisor)
   {
      super(divisor);
   }

   public abstract YoRegistry getRegistry();
}
