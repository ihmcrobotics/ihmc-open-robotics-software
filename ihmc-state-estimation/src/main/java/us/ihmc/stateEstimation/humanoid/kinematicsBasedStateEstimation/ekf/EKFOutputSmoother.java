package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.euclid.filters.RateLimitedYoFramePoint3D;
import us.ihmc.yoVariables.euclid.filters.RateLimitedYoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Controller-facing localization correction. Never updates EKF covariance or invents physical velocity.
 * <p>
 * Each tick, the output first moves with the kinematic pose, so all physical motion passes through unchanged. Then its position
 * and orientation are rate limited toward the EKF's estimate, so only the localization correction is limited to
 * {@code ekfMaximumCorrectionTranslationSpeed} and {@code ekfMaximumCorrectionRotationSpeed}.
 * </p>
 */
public final class EKFOutputSmoother
{
   private final YoDouble maximumTranslationSpeed;
   private final YoDouble maximumRotationSpeed;
   private final RateLimitedYoFramePoint3D outputPosition;
   private final RateLimitedYoFrameQuaternion outputOrientation;
   private final RigidBodyTransform output = new RigidBodyTransform();
   private final RigidBodyTransform previousKinematicPose = new RigidBodyTransform();
   private final RigidBodyTransform kinematicMotion = new RigidBodyTransform();
   private final Quaternion orientationInput = new Quaternion();
   private boolean initialized;

   /**
    * @param maximumTranslationSpeed the initial maximum speed of the position correction, in m/s.
    * @param maximumRotationSpeed    the initial maximum speed of the orientation correction, in rad/s.
    * @param dt                      the time between two calls to {@link #update}, in seconds: the estimator period.
    * @param registry                the registry the speed limits and the smoothed pose are added to.
    */
   public EKFOutputSmoother(double maximumTranslationSpeed, double maximumRotationSpeed, double dt, YoRegistry registry)
   {
      if (!Double.isFinite(maximumTranslationSpeed) || maximumTranslationSpeed <= 0 || !Double.isFinite(maximumRotationSpeed) || maximumRotationSpeed <= 0)
         throw new IllegalArgumentException("Output correction speeds must be finite and positive");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      this.maximumTranslationSpeed = new YoDouble("ekfMaximumCorrectionTranslationSpeed", registry);
      this.maximumRotationSpeed = new YoDouble("ekfMaximumCorrectionRotationSpeed", registry);
      this.maximumTranslationSpeed.set(maximumTranslationSpeed);
      this.maximumRotationSpeed.set(maximumRotationSpeed);
      outputPosition = new RateLimitedYoFramePoint3D("ekfSmoothedPelvisPosition", "", registry, this.maximumTranslationSpeed, dt, worldFrame);
      outputOrientation = new RateLimitedYoFrameQuaternion("ekfSmoothedPelvisOrientation", "", registry, this.maximumRotationSpeed, dt, worldFrame);
   }

   public void reset()
   {
      initialized = false;
      outputPosition.reset();
      outputOrientation.reset();
   }

   /**
    * @param kinematicPose the kinematic pose estimate this tick. Not modified.
    * @param target        the EKF's estimate this tick, which the output is rate limited toward. Not modified.
    * @param resultToPack        the smoothed pose. Modified.
    */
   public void update(RigidBodyTransformReadOnly kinematicPose, RigidBodyTransformReadOnly target, RigidBodyTransform resultToPack)
   {
      if (!isFinite(kinematicPose) || !isFinite(target))
         throw new IllegalArgumentException("Output smoother requires finite poses");
      if (!initialized)
      {
         // The first update starts the output at the kinematic pose.
         outputPosition.update(kinematicPose.getTranslation());
         orientationInput.set(kinematicPose.getRotation());
         outputOrientation.update(orientationInput);
         initialized = true;
      }
      else
      {
         // Preserve all physical KIO motion; limit only localization change relative to this prediction.
         kinematicMotion.setAndInvert(previousKinematicPose);
         kinematicMotion.multiply(kinematicPose);
         output.set(outputOrientation, outputPosition);
         output.multiply(kinematicMotion);
         outputPosition.set(output.getTranslation());
         outputOrientation.set(output.getRotation());

         outputPosition.update(target.getTranslation());
         orientationInput.set(target.getRotation());
         outputOrientation.update(orientationInput);
      }
      previousKinematicPose.set(kinematicPose);
      resultToPack.set(outputOrientation, outputPosition);
   }

   private static boolean isFinite(RigidBodyTransformReadOnly pose)
   {
      return pose != null && !pose.containsNaN();
   }
}
