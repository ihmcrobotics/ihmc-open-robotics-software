package us.ihmc.avatar.factory;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class SimulatedHandControlTask extends HumanoidRobotControlTask
{
   public SimulatedHandControlTask(long divisor)
   {
      super(divisor);
   }

   public abstract YoVariableRegistry getRegistry();
}
