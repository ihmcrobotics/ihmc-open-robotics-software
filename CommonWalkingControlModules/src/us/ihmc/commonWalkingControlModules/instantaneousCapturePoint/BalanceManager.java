package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class BalanceManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public BalanceManager(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      
   }
}
