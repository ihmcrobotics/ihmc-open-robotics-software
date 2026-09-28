package us.ihmc.stateEstimation.invariantEstimator;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The invariant filter's accelerometer bias, estimated ONCE from a stretch of standing still after start-up
 * and then held -- the rule the offline evaluation uses (per session, from its stationary prefix).
 * <p>
 * Why it exists: the invariant filter subtracts whatever accelerometer bias its joint-level provider
 * publishes, and the joint-level KF publishes zero by design (it estimates gyro biases only). On Alex the
 * pelvis IMU reads |a| = 9.56-9.58 m/s^2 at rest, and without a correction the filter integrates that
 * into a steady -0.03 m/s vertical velocity which the contact updates hold the position against.
 * <p>
 * The estimate is the error ALONG gravity, {@code (|a| - g) a / |a|} for the mean raw specific force
 * {@code a} over the window: at rest a horizontal bias is indistinguishable from a tilt of the IMU, and the
 * gravity-leveling update owns tilt. Samples accumulate only while the raw gyro stays below a stillness
 * threshold; any motion restarts the window. Until the window completes the bias is zero, and
 * {@code staticAccelBiasState} says so. Setting {@code staticAccelBiasReestimate} true (with the robot
 * standing still) discards the held value and collects a new one.
 * <p>
 * Allocation-free per tick.
 */
public class StaticAccelerometerBiasEstimator
{
   public enum State
   {
      COLLECTING, HELD
   }

   public static final double DEFAULT_STILL_GYRO_NORM = 0.05; // rad/s

   private final double gravityMagnitude;
   private final double windowSeconds;
   private final double stillGyroNorm;

   private final YoEnum<State> state;
   private final YoDouble collectedSeconds;
   private final YoInteger restarts;
   private final YoDouble meanSpecificForceNorm;
   private final YoDouble biasX, biasY, biasZ;
   private final YoBoolean reestimate;

   private final Vector3D sum = new Vector3D();
   private final Vector3D bias = new Vector3D();
   private int samples = 0;

   /**
    * @param gravityMagnitude |g| the corrected accelerometer should read at rest, m/s^2 (sign ignored).
    * @param windowSeconds    seconds of continuous stillness to average over; must be positive.
    * @param stillGyroNorm    raw gyro norm (rad/s) above which the robot counts as moving.
    */
   public StaticAccelerometerBiasEstimator(double gravityMagnitude, double windowSeconds, double stillGyroNorm, YoRegistry parentRegistry)
   {
      if (!(windowSeconds > 0.0))
         throw new IllegalArgumentException("windowSeconds must be positive, got " + windowSeconds);
      this.gravityMagnitude = Math.abs(gravityMagnitude);
      this.windowSeconds = windowSeconds;
      this.stillGyroNorm = stillGyroNorm;

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      state = new YoEnum<>("staticAccelBiasState", registry, State.class);
      collectedSeconds = new YoDouble("staticAccelBiasCollectedSeconds", registry);
      restarts = new YoInteger("staticAccelBiasRestarts", registry);
      meanSpecificForceNorm = new YoDouble("staticAccelBiasMeanSpecificForceNorm", registry);
      biasX = new YoDouble("staticAccelBiasX", registry);
      biasY = new YoDouble("staticAccelBiasY", registry);
      biasZ = new YoDouble("staticAccelBiasZ", registry);
      reestimate = new YoBoolean("staticAccelBiasReestimate", registry);
      state.set(State.COLLECTING);
      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   /**
    * @param rawSpecificForce raw accelerometer reading in the IMU measurement frame (no bias removed).
    * @param rawGyroNorm      norm of the raw gyro reading, rad/s.
    * @param dt               tick period, s.
    */
   public void update(Tuple3DReadOnly rawSpecificForce, double rawGyroNorm, double dt)
   {
      if (reestimate.getValue())
      {
         reestimate.set(false);
         restartWindow();
         bias.setToZero();
         publishBias();
         state.set(State.COLLECTING);
      }
      if (state.getValue() == State.HELD)
         return;

      if (!(rawGyroNorm <= stillGyroNorm) || !Double.isFinite(rawSpecificForce.getX() + rawSpecificForce.getY() + rawSpecificForce.getZ()))
      {
         if (samples > 0)
            restarts.increment();
         restartWindow();
         return;
      }

      sum.add(rawSpecificForce);
      samples++;
      collectedSeconds.set(samples * dt);
      if (samples * dt >= windowSeconds)
      {
         bias.setAndScale(1.0 / samples, sum);
         double norm = bias.norm();
         meanSpecificForceNorm.set(norm);
         bias.scale((norm - gravityMagnitude) / norm);
         publishBias();
         state.set(State.HELD);
         // Once per estimate, not per tick: the operator's confirmation that the robot stood still long enough.
         us.ihmc.log.LogTools.info(String.format("Accelerometer bias held after %.1f s standing: |a| = %.4f m/s^2, bias = (%.4f, %.4f, %.4f)",
                                                 samples * dt, norm, bias.getX(), bias.getY(), bias.getZ()));
      }
   }

   /** Zero until the window has completed; then the held estimate, IMU measurement frame. */
   public Vector3DReadOnly getBias()
   {
      return bias;
   }

   public State getState()
   {
      return state.getValue();
   }

   private void restartWindow()
   {
      sum.setToZero();
      samples = 0;
      collectedSeconds.set(0.0);
   }

   private void publishBias()
   {
      biasX.set(bias.getX());
      biasY.set(bias.getY());
      biasZ.set(bias.getZ());
   }
}
