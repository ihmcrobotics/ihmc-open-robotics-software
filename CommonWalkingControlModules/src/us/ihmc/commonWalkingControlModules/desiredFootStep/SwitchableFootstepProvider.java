package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.humanoidRobotics.footstep.Footstep;

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

   @Override
   public Footstep poll()
   {
      return getFootstepProviderInUse().poll();
   }

   @Override
   public Footstep peek()
   {
      return getFootstepProviderInUse().peek();
   }

   @Override
   public Footstep peekPeek()
   {
      return getFootstepProviderInUse().peekPeek();
   }

   @Override
   public boolean isEmpty()
   {
      return getFootstepProviderInUse().isEmpty();
   }

   @Override
   public void notifyComplete(FramePose actualFootPoseInWorld)
   {
      getFootstepProviderInUse().notifyComplete(actualFootPoseInWorld);
   }

   @Override
   public void notifyWalkingComplete()
   {
      getFootstepProviderInUse().notifyWalkingComplete();
   }

   @Override
   public int getNumberOfFootstepsToProvide()
   {
      return getFootstepProviderInUse().getNumberOfFootstepsToProvide();
   }

   @Override
   public boolean isBlindWalking()
   {
      FootstepProvider footstepProviderInUse = getFootstepProviderInUse();
      return footstepProviderInUse.isBlindWalking();
   }

   @Override
   public boolean isPaused()
   {
      return false;
   }

   @Override
   public void cancelPlan()
   {
      getFootstepProviderInUse().cancelPlan();
   }
}
