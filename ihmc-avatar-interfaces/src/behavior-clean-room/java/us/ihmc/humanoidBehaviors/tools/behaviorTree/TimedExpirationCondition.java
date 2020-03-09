package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import us.ihmc.commons.time.Stopwatch;

import java.util.function.DoubleSupplier;

/**
 * A timed expiration behavrior tree condition action that is updatable but expires with wall time.
 */
public class TimedExpirationCondition extends BehaviorTreeCondition
{
   private final Stopwatch stopwatch = new Stopwatch();
   private final DoubleSupplier expirationTimeSupplier;
   private final Double expirationTime;

   private boolean hasBeenRenewed = false; // initial condition is failure

   public TimedExpirationCondition(DoubleSupplier expirationTimeSupplier)
   {
      super(null);
      this.expirationTimeSupplier = expirationTimeSupplier;
      this.expirationTime = null;
   }

   public TimedExpirationCondition(double expirationTime)
   {
      super(null);
      this.expirationTime = expirationTime;
      this.expirationTimeSupplier = null;
   }

   public void update()
   {
      hasBeenRenewed = true;
      stopwatch.reset();
   }

   private double getExpirationTime()
   {
      if (expirationTime == null)
      {
         return expirationTimeSupplier.getAsDouble();
      }
      else
      {
         return expirationTime;
      }
   }

   @Override
   protected boolean checkCondition()
   {
      if (!hasBeenRenewed)
         return false;

      double elapsed = stopwatch.lapElapsed();
      boolean expired = elapsed >= getExpirationTime();
      boolean success = !expired;
      return success;
   }
}
