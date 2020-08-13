package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.CurrentState;

public class BehaviorStateReference<E extends Enum<E>>
{
   private final AtomicReference<E> atomicReference;
   private final StatusLogger statusLogger;
   private final UIPublisher uiPublisher;

   public BehaviorStateReference(E initialValue, StatusLogger statusLogger, UIPublisher uiPublisher)
   {
      atomicReference = new AtomicReference<>(initialValue);
      this.statusLogger = statusLogger;
      this.uiPublisher = uiPublisher;
   }

   public void set(E state)
   {
      atomicReference.set(state);
      statusLogger.info(1, "Entering state: " + get().name());
      broadcast();
   }

   public void broadcast()
   {
      uiPublisher.publishToUI(CurrentState, atomicReference.get().name());
   }

   public E get()
   {
      return atomicReference.get();
   }
}
