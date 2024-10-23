package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This filter aims at merging two measurements:
 * <ul>
 * <li>position measurement that is drift free but contains high frequency noise.
 * <li>rate measurement that can be integrated into a less noisy position measurement but is thus
 * sensitive to bias accumulation.
 * </ul>
 * A single update to this filter does the following:
 *
 * <pre>
 * xDot_est^{n+1}  = xDot_meas + xDot_bias^{n}
 * x_prediction    = x_est^{n} + 0.5 * (xDot_est^{n} + xDot_est^{n+1}) * dt
 * x_error         = x_meas - x_pred
 * x_est^{n+1}     = x_prediction + kp * x_error
 * xDot_bias^{n+1} = xDot_bias^{n} + ki * x_error
 * </pre>
 *
 * In words, the filter integrates the rate measurement to compute the position estimate and uses
 * the position measurement to continuously correct for the bias in the rate.
 *
 * @author Sylvain
 */
public class IntegratorBiasCompensatorYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final double dt;
   private final DoubleProvider kp, ki;
   private final DoubleProvider rawPosition;
   private final DoubleProvider rawRate;
   private final YoDouble error;
   private final YoDouble estimatedRate;
   private final YoDouble estimatedRateBias;
   private final YoBoolean hasBeenCalled;

   public static DoubleProvider createKpYoDouble(String namePrefix, double initialValue, YoRegistry registry)
   {
      YoDouble kp = new YoDouble(namePrefix + "FilterKp", registry);
      kp.set(initialValue);
      return kp;
   }

   public static DoubleProvider createKiYoDouble(String namePrefix, double initialValue, YoRegistry registry)
   {
      YoDouble ki = new YoDouble(namePrefix + "FilterKi", registry);
      ki.set(initialValue);
      return ki;
   }

   public IntegratorBiasCompensatorYoVariable(String name,
                                              YoRegistry registry,
                                              double kp,
                                              double ki,
                                              DoubleProvider rawPositionVariable,
                                              DoubleProvider rawRateVariable,
                                              double dt)
   {
      this(name, registry, createKpYoDouble(name, kp, registry), createKiYoDouble(name, ki, registry), rawPositionVariable, rawRateVariable, dt);
   }

   public IntegratorBiasCompensatorYoVariable(String name,
                                              YoRegistry registry,
                                              DoubleProvider kp,
                                              DoubleProvider ki,
                                              DoubleProvider rawPositionVariable,
                                              DoubleProvider rawRateVariable,
                                              double dt)
   {
      super(name, registry);
      this.kp = kp;
      this.ki = ki;
      this.dt = dt;
      this.rawPosition = rawPositionVariable;
      this.rawRate = rawRateVariable;
      this.error = new YoDouble(name + "PositionError", registry);
      this.estimatedRate = new YoDouble(name + "EstimatedRate", registry);
      this.estimatedRateBias = new YoDouble(name + "EstimatedRateBias", registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);
      reset();
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public YoDouble getPositionEstimation()
   {
      return this;
   }

   public YoDouble getRateEstimation()
   {
      return estimatedRate;
   }

   public YoDouble getBiasEstimation()
   {
      return estimatedRateBias;
   }

   @Override
   public void update()
   {
      update(rawPosition.getValue(), rawRate.getValue());
   }

   public void update(double rawPosition, double rawRate)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(rawPosition);
         estimatedRate.set(rawRate);
         error.set(0.0);
         estimatedRateBias.set(0.0);
         return;
      }

      double x_filt = this.getValue();

      double xd_filt_new = rawRate + estimatedRateBias.getValue();
      double xd_filt_old = estimatedRate.getValue();
      double x_pred = x_filt + 0.5 * (xd_filt_old + xd_filt_new) * dt;
      double error = rawPosition - x_pred;
      x_filt = x_pred + kp.getValue() * error;
      this.error.set(error);
      estimatedRateBias.add(ki.getValue() * error);
      estimatedRate.set(rawRate + estimatedRateBias.getValue());
      set(x_filt);
   }
}