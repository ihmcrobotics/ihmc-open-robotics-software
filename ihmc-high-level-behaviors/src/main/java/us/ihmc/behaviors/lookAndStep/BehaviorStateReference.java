package us.ihmc.behaviors.lookAndStep;

import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;

import java.util.concurrent.atomic.AtomicReference;

public class BehaviorStateReference<E extends Enum<E>>
{
   private final AtomicReference<E> atomicReference;
   private final StatusLogger statusLogger;
   private final BehaviorHelper behaviorHelper;

   public BehaviorStateReference(E initialValue, StatusLogger statusLogger, BehaviorHelper behaviorHelper)
   {
      atomicReference = new AtomicReference<>(initialValue);
      this.statusLogger = statusLogger;
      this.behaviorHelper = behaviorHelper;
   }

   public void set(E state)
   {
      atomicReference.set(state);
      statusLogger.info(1, "Entering state: " + get().name());
      broadcast();
   }

   public void broadcast()
   {
      behaviorHelper.publish(LookAndStepBehaviorAPI.CURRENT_STATE, atomicReference.get().name());
   }

   public E get()
   {
      return atomicReference.get();
   }
}
