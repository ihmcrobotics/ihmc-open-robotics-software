package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;

public class PoseDerivativeCalculator
{
   private double previousTime;
   private double nextTime;
   private final Pose3D previousPose = new Pose3D();
   private final Pose3D nextPose = new Pose3D();
   private final Vector3D linearVelocity = new Vector3D();

   public void reset()
   {
      previousTime = Double.NaN;
      nextTime = Double.NaN;
      previousPose.setToNaN();
      nextPose.setToNaN();
      linearVelocity.setToZero();
   }

   /** @return If result is valid yet. */
   public boolean compute(Pose3DReadOnly poseUpdate, double timeUpdate)
   {
      nextTime = timeUpdate;
      nextPose.set(poseUpdate);
      boolean pastFirstUpdate = !previousPose.containsNaN();
      boolean timeHasPassed = false;

      if (pastFirstUpdate)
      {
         double dt = nextTime - previousTime;
         boolean dtNonZero = dt > 0.0;
         boolean poseChanged = !nextPose.getPosition().geometricallyEquals(previousPose.getPosition(), 1e-8);
         timeHasPassed = dtNonZero;
         timeHasPassed &= poseChanged;

         if (dtNonZero && !poseChanged)
         {
            LogTools.warn(("Dt was %.6f but no position change."
                         + "If this happens on real robot there's a problem").formatted(dt)); // State estimator time may pass without a simulation tick
         }

         if (timeHasPassed)
         {
            linearVelocity.sub(nextPose.getPosition(), previousPose.getPosition());
            linearVelocity.scale(1.0 / dt);
         }
      }

      boolean shouldSkipUpdatingPrevious = pastFirstUpdate && !timeHasPassed;

      if (!shouldSkipUpdatingPrevious)
      {
         previousTime = nextTime;
         previousPose.set(nextPose);
      }

      return pastFirstUpdate && timeHasPassed;
   }

   public Vector3D getLinearVelocity()
   {
      return linearVelocity;
   }
}
