package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class IntegratorBiasCompensatorYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final double dt;
   private final DoubleProvider kp, ki;
   private final YoDouble rawPosition;
   private final YoDouble rawRate;
   private final YoDouble error, biasEstimate;
   private final YoDouble estimatedRate;
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
                                              YoDouble rawPositionVariable,
                                              YoDouble rawRateVariable,
                                              double dt)
   {
      this(name, registry, createKpYoDouble(name, kp, registry), createKiYoDouble(name, ki, registry), rawPositionVariable, rawRateVariable, dt);
   }

   public IntegratorBiasCompensatorYoVariable(String name,
                                              YoRegistry registry,
                                              DoubleProvider kp,
                                              DoubleProvider ki,
                                              YoDouble rawPositionVariable,
                                              YoDouble rawRateVariable,
                                              double dt)
   {
      super(name, registry);
      this.kp = kp;
      this.ki = ki;
      this.dt = dt;
      this.rawPosition = rawPositionVariable;
      this.rawRate = rawRateVariable;
      this.error = new YoDouble(name + "PositionError", registry);
      this.biasEstimate = new YoDouble(name + "BiasEstimate", registry);
      this.estimatedRate = new YoDouble(name + "EstimatedRate", registry);
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
         biasEstimate.set(0.0);
         return;
      }

      double kp = this.kp.getValue();
      double ki = this.ki.getValue();
      double x_meas = rawPosition;
      double xd_meas = rawRate;
      double x_filt = this.getValue();
      double xd_filt = xd_meas + biasEstimate.getValue();
      double x_pred = x_filt + xd_filt * dt;
      double error = x_meas - x_pred;
      x_filt = x_pred + kp * error;
      this.error.set(error);
      biasEstimate.add(ki * error);
      set(x_filt);
      estimatedRate.set(xd_meas + biasEstimate.getValue());
   }
}