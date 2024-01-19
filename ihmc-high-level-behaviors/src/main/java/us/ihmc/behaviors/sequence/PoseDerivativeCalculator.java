package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

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

      // Be robust to receiving a pose that's the same as the last
      boolean poseChanged = !nextPose.getPosition().geometricallyEquals(previousPose.getPosition(), 1e-11);

      if (pastFirstUpdate && poseChanged)
      {
         double dt = nextTime - previousTime;

         linearVelocity.sub(nextPose.getPosition(), previousPose.getPosition());
         linearVelocity.scale(1.0 / dt);
      }

      boolean shouldSkipUpdatingPrevious = pastFirstUpdate && !poseChanged;

      if (!shouldSkipUpdatingPrevious)
      {
         previousTime = nextTime;
         previousPose.set(nextPose);
      }

      return pastFirstUpdate;
   }

   public Vector3D getLinearVelocity()
   {
      return linearVelocity;
   }
}
