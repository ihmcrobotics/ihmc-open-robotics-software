package us.ihmc.stateEstimation.invariantEstimator;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The primary IMU's gyro bias, re-measured every time the robot stands on both feet, and held in between.
 * <p>
 * Why it exists: on Alex002 (2026-10-06 logs) the joint-level KF drives the pelvis gyro bias to +20 mrad/s in
 * pitch (the clamp) and about -15 mrad/s in yaw within a minute of walking, while the raw gyro averages under
 * 1 mrad/s at every standing rest after warm-up. That residual rate times the pelvis-to-sole lever is the 1-2 cm/s
 * standing velocity bias and the extra tilt of configurations 4 and 6. A bias held from start-up is no better: the
 * same gyro reads 7-17 mrad/s in pitch for the first ~2 minutes after power-up, decaying to ~0, while the
 * accelerometer shows no rotation at all (a warm-up transient).
 * <p>
 * The estimate over a rest of length T is
 * <pre>
 *   b = ( sum(omega_raw) dt - dTheta ) / T
 * </pre>
 * where dTheta is the IMU's rotation over the rest relative to the soles, from encoder kinematics (the rotation
 * vector of R_start^T R_now, averaged over the two feet). With both feet planted the soles do not rotate, so the
 * true angular rate integrates to dTheta and whatever is left is bias; subtracting it stops a slow lean or squat
 * from leaking into the bias. A rest is both feet loaded and a raw gyro norm under a threshold; any tick outside it
 * ends the rest. The bias is published at every {@code windowSeconds} of continuous rest (from the whole rest so
 * far), so the lift-off at the end of a rest is never in a published estimate by more than a fraction of a window.
 * Until the first rest completes a window the bias is zero.
 * <p>
 * Allocation-free per tick.
 */
public class RestGyroBiasEstimator
{
   public static final double DEFAULT_WINDOW_SECONDS = 2.0;
   public static final double DEFAULT_MAX_GYRO_NORM = 0.1; // rad/s; standing sway peaks at ~0.05

   private final double windowSeconds;
   private final double maxGyroNorm;

   private final YoDouble biasX, biasY, biasZ;
   private final YoInteger updates;
   private final YoDouble restSeconds;
   private final YoDouble lastKinematicRotation;
   private final YoDouble lastRawMeanNorm;
   private final YoBoolean resting;

   private final Vector3D sum = new Vector3D();
   private final Vector3D bias = new Vector3D();
   private final Vector3D rotation = new Vector3D();
   private final Vector3D sideRotation = new Vector3D();
   private final Quaternion[] startOrientations = {new Quaternion(), new Quaternion()};
   private final Quaternion relative = new Quaternion();
   private int samples = 0;
   private double nextPublishSeconds;

   public RestGyroBiasEstimator(YoRegistry parentRegistry)
   {
      this(DEFAULT_WINDOW_SECONDS, DEFAULT_MAX_GYRO_NORM, parentRegistry);
   }

   /**
    * @param windowSeconds seconds of continuous rest between published estimates; must be positive.
    * @param maxGyroNorm   raw gyro norm (rad/s) above which the robot is not resting.
    */
   public RestGyroBiasEstimator(double windowSeconds, double maxGyroNorm, YoRegistry parentRegistry)
   {
      if (!(windowSeconds > 0.0))
         throw new IllegalArgumentException("windowSeconds must be positive, got " + windowSeconds);
      this.windowSeconds = windowSeconds;
      this.maxGyroNorm = maxGyroNorm;
      nextPublishSeconds = windowSeconds;

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      biasX = new YoDouble("restGyroBiasInIMUFrameX", registry);
      biasY = new YoDouble("restGyroBiasInIMUFrameY", registry);
      biasZ = new YoDouble("restGyroBiasInIMUFrameZ", registry);
      updates = new YoInteger("restGyroBiasUpdates", registry);
      restSeconds = new YoDouble("restGyroBiasRestSeconds", registry);
      lastKinematicRotation = new YoDouble("restGyroBiasLastKinematicRotation", registry);
      lastRawMeanNorm = new YoDouble("restGyroBiasLastRawMeanNorm", registry);
      resting = new YoBoolean("restGyroBiasResting", registry);
      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   /**
    * @param rawAngularRate raw gyro reading in the IMU measurement frame (no bias removed), rad/s.
    * @param bothFeetLoaded true while both feet are in contact.
    * @param imuInLeftSole  orientation of the IMU measurement frame in the left sole frame, from encoders.
    * @param imuInRightSole the same for the right sole.
    * @param dt             tick period, s.
    */
   public void update(Tuple3DReadOnly rawAngularRate,
                      boolean bothFeetLoaded,
                      Orientation3DReadOnly imuInLeftSole,
                      Orientation3DReadOnly imuInRightSole,
                      double dt)
   {
      double norm = Math.sqrt(rawAngularRate.getX() * rawAngularRate.getX() + rawAngularRate.getY() * rawAngularRate.getY()
                              + rawAngularRate.getZ() * rawAngularRate.getZ());
      if (!bothFeetLoaded || !(norm <= maxGyroNorm))
      {
         endRest();
         return;
      }

      if (samples == 0)
      {
         startOrientations[0].set(imuInLeftSole);
         startOrientations[1].set(imuInRightSole);
         nextPublishSeconds = windowSeconds;
      }
      sum.add(rawAngularRate);
      samples++;
      double elapsed = samples * dt;
      resting.set(true);
      restSeconds.set(elapsed);

      if (elapsed >= nextPublishSeconds)
      {
         rotation.setToZero();
         accumulateRotation(startOrientations[0], imuInLeftSole);
         accumulateRotation(startOrientations[1], imuInRightSole);
         rotation.scale(0.5);
         lastKinematicRotation.set(rotation.norm());
         lastRawMeanNorm.set(sum.norm() / samples);

         bias.setAndScale(dt, sum);
         bias.sub(rotation);
         bias.scale(1.0 / elapsed);
         biasX.set(bias.getX());
         biasY.set(bias.getY());
         biasZ.set(bias.getZ());
         updates.increment();
         nextPublishSeconds += windowSeconds;
      }
   }

   /** Rotation vector of start^-1 * now, i.e. the rotation over the rest expressed in the IMU frame at its start. */
   private void accumulateRotation(Quaternion start, Orientation3DReadOnly now)
   {
      relative.set(now);
      relative.preMultiplyConjugateOther(start);
      relative.getRotationVector(sideRotation);
      rotation.add(sideRotation);
   }

   private void endRest()
   {
      sum.setToZero();
      samples = 0;
      resting.set(false);
      restSeconds.set(0.0);
   }

   /** Zero until the first rest completes a window; then the latest estimate, IMU measurement frame. */
   public Vector3DReadOnly getBias()
   {
      return bias;
   }

   public int getNumberOfUpdates()
   {
      return updates.getValue();
   }
}
