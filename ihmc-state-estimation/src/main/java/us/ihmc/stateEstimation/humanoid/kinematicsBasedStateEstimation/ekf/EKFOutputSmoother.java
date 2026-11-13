package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

/** Controller-facing localization correction. Never updates EKF covariance or invents physical velocity. */
public final class EKFOutputSmoother
{
   private final double maximumTranslationSpeed, maximumRotationSpeed;
   private final RigidBodyTransform output = new RigidBodyTransform(), previousRaw = new RigidBodyTransform(), motion = new RigidBodyTransform();
   private final Vector3D translation = new Vector3D(), angle = new Vector3D();
   private final Quaternion rotation = new Quaternion(), increment = new Quaternion(), orientation = new Quaternion();
   private boolean initialized;
   private long previousTime;

   public EKFOutputSmoother(double maximumTranslationSpeed, double maximumRotationSpeed)
   {
      if (!Double.isFinite(maximumTranslationSpeed) || maximumTranslationSpeed <= 0 || !Double.isFinite(maximumRotationSpeed) || maximumRotationSpeed <= 0)
         throw new IllegalArgumentException("Output correction speeds must be finite and positive");
      this.maximumTranslationSpeed = maximumTranslationSpeed;
      this.maximumRotationSpeed = maximumRotationSpeed;
   }

   public void reset()
   {
      initialized = false;
      previousTime = 0;
   }

   public void update(long time, RigidBodyTransformReadOnly raw, RigidBodyTransformReadOnly target, RigidBodyTransform result)
   {
      if (!isFinite(raw) || !isFinite(target))
         throw new IllegalArgumentException("Output smoother requires finite poses");
      if (!initialized)
      {
         output.set(raw);
         initialized = true;
      }
      else
      {
         if (time <= previousTime)
            throw new IllegalArgumentException("Output smoother requires increasing monotonic time");
         double dt = (time - previousTime) * 1e-9;
         // Preserve all physical KIO motion; limit only localization change relative to this prediction.
         motion.set(previousRaw);
         motion.invert();
         motion.multiply(raw);
         output.multiply(motion);

         translation.set(target.getTranslation());
         translation.sub(output.getTranslation());
         double distance = translation.length();
         if (distance > 0)
         {
            translation.scale(Math.min(1, maximumTranslationSpeed * dt / distance));
            output.getTranslation().add(translation);
         }

         rotation.set(target.getRotation());
         orientation.set(output.getRotation());
         rotation.multiplyConjugateOther(orientation);
         rotation.normalizeAndLimitToPi();
         rotation.getRotationVector(angle);
         double magnitude = angle.length();
         if (magnitude > 0)
         {
            angle.scale(Math.min(1, maximumRotationSpeed * dt / magnitude));
            increment.setRotationVector(angle);
            orientation.set(output.getRotation());
            orientation.preMultiply(increment);
            orientation.normalize();
            output.getRotation().set(orientation);
         }
      }
      previousRaw.set(raw);
      previousTime = time;
      result.set(output);
   }

   private static boolean isFinite(RigidBodyTransformReadOnly pose)
   {
      return pose != null && !pose.containsNaN();
   }
}
