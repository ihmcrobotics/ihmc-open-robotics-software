package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;


public class SwitchableFootstepProvider implements FootstepProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final IntegerYoVariable footstepProviderIndex = new IntegerYoVariable("footstepProviderIndex", registry);

   private final ArrayList<FootstepProvider> footstepProviders = new ArrayList<FootstepProvider>();

   public SwitchableFootstepProvider(FootstepProvider footstepProvider, YoVariableRegistry parentRegistry)
   {
      footstepProviders.add(footstepProvider);
      parentRegistry.addChild(registry);
   }

   public void addFootstepProvider(FootstepProvider footstepProvider)
   {
      footstepProviders.add(footstepProvider);
   }

   private FootstepProvider getFootstepProviderInUse()
   {
      int index = footstepProviderIndex.getIntegerValue();
      if (index < 0)
         index = 0;
      if (index >= footstepProviders.size())
         index = 0;

      FootstepProvider footstepProvider = footstepProviders.get(index);

      return footstepProvider;
   }

   public Footstep poll()
   {
      return getFootstepProviderInUse().poll();
   }

   public Footstep peek()
   {
      return getFootstepProviderInUse().peek();
   }

   public Footstep peekPeek()
   {
      return getFootstepProviderInUse().peekPeek();
   }

   public boolean isEmpty()
   {
      return getFootstepProviderInUse().isEmpty();
   }

   public void notifyComplete()
   {
      getFootstepProviderInUse().notifyComplete();
   }

   public void notifyWalkingComplete()
   {
      getFootstepProviderInUse().notifyWalkingComplete();
   }

   public int getNumberOfFootstepsToProvide()
   {
      return getFootstepProviderInUse().getNumberOfFootstepsToProvide();
   }

   public boolean isBlindWalking()
   {
      FootstepProvider footstepProviderInUse = getFootstepProviderInUse();
      return footstepProviderInUse.isBlindWalking();
   }

}
