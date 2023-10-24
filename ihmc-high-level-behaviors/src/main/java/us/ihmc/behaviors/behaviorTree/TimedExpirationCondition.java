package us.ihmc.behaviors.behaviorTree;

import us.ihmc.commons.time.Stopwatch;

import java.util.function.DoubleSupplier;

/**
 * A timed expiration behavrior tree condition action that is updatable but expires with wall time.
 */
public class TimedExpirationCondition extends BehaviorTreeCondition
{
   private final Stopwatch stopwatch = new Stopwatch();
   private final DoubleSupplier expirationDurationSupplier;
   private final Double expirationDuration;

   private boolean hasBeenRenewed = false; // initial condition is failure

   public TimedExpirationCondition(DoubleSupplier expirationDurationSupplier)
   {
      super(null);
      this.expirationDurationSupplier = expirationDurationSupplier;
      this.expirationDuration = null;
   }

   public TimedExpirationCondition(double expirationDuration)
   {
      super(null);
      this.expirationDuration = expirationDuration;
      this.expirationDurationSupplier = null;
   }

   public void update()
   {
      hasBeenRenewed = true;
      stopwatch.reset();
   }

   private double getExpirationDuration()
   {
      if (expirationDuration == null)
      {
         return expirationDurationSupplier.getAsDouble();
      }
      else
      {
         return expirationDuration;
      }
   }

   @Override
   public boolean checkCondition()
   {
      if (!hasBeenRenewed)
         return false;

      double elapsed = stopwatch.lapElapsed();
      boolean expired = elapsed >= getExpirationDuration();
      boolean success = !expired;
      return success;
   }
}
