package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class YoAbortWalkingProvider implements AbortWalkingProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final BooleanYoVariable userAbortWalking = new BooleanYoVariable("userAbortWalking", registry);

   public YoAbortWalkingProvider(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public boolean shouldAbortWalking()
   {
      return userAbortWalking.getBooleanValue();
   }

   public void walkingAborted()
   {
      userAbortWalking.set(false);
   }

   public void triggerAbort()
   {
      userAbortWalking.set(true);
   }
}
