package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.math.filters.IntegratorBiasCompensatorYoVariable.createKiYoDouble;
import static us.ihmc.robotics.math.filters.IntegratorBiasCompensatorYoVariable.createKpYoDouble;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class IntegratorBiasCompensatorYoFrameVector3D extends YoFrameVector3D implements ProcessingYoVariable
{
   private final double dt;
   private final DoubleProvider kp, ki;
   private final FrameTuple3DReadOnly rawPosition;
   private final FrameTuple3DReadOnly rawRate;
   private final YoFrameVector3D error, biasEstimate;
   private final YoFrameVector3D estimatedRate;
   private final YoBoolean hasBeenCalled;

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   double kp,
                                                   double ki,
                                                   FrameTuple3DReadOnly rawPositionVariable,
                                                   FrameTuple3DReadOnly rawRateVariable,
                                                   double dt)
   {
      this(name, registry, createKpYoDouble(name, kp, registry), createKiYoDouble(name, ki, registry), rawPositionVariable, rawRateVariable, dt);
   }

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   DoubleProvider kp,
                                                   DoubleProvider ki,
                                                   ReferenceFrame referenceFrame,
                                                   double dt)
   {
      this(name, registry, kp, ki, null, null, referenceFrame, dt);
   }

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   DoubleProvider kp,
                                                   DoubleProvider ki,
                                                   FrameTuple3DReadOnly rawPositionVariable,
                                                   FrameTuple3DReadOnly rawRateVariable,
                                                   double dt)
   {
      this(name, registry, kp, ki, rawPositionVariable, rawRateVariable, rawPositionVariable.getReferenceFrame(), dt);
   }

   private IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                    YoRegistry registry,
                                                    DoubleProvider kp,
                                                    DoubleProvider ki,
                                                    FrameTuple3DReadOnly rawPositionVariable,
                                                    FrameTuple3DReadOnly rawRateVariable,
                                                    ReferenceFrame referenceFrame,
                                                    double dt)
   {
      super(name, referenceFrame, registry);

      if (rawPositionVariable != null && rawRateVariable != null)
         rawPositionVariable.checkReferenceFrameMatch(rawRateVariable);

      this.kp = kp;
      this.ki = ki;
      this.dt = dt;
      this.rawPosition = rawPositionVariable;
      this.rawRate = rawRateVariable;
      this.error = new YoFrameVector3D(name + "PositionError", referenceFrame, registry);
      this.biasEstimate = new YoFrameVector3D(name + "BiasEstimate", referenceFrame, registry);
      this.estimatedRate = new YoFrameVector3D(name + "EstimatedRate", referenceFrame, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);
      reset();
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public YoFrameVector3D getPositionEstimation()
   {
      return this;
   }

   public YoFrameVector3D getRateEstimation()
   {
      return estimatedRate;
   }

   @Override
   public void update()
   {
      update(rawPosition, rawRate);
   }

   private final Vector3D xd_filt = new Vector3D();
   private final Vector3D x_pred = new Vector3D();

   public void update(FrameTuple3DReadOnly rawPosition, FrameTuple3DReadOnly rawRate)
   {
      checkReferenceFrameMatch(rawPosition, rawRate);
      update((Tuple3DReadOnly) rawPosition, (Tuple3DReadOnly) rawRate);
   }

   public void update(Tuple3DReadOnly rawPosition, Tuple3DReadOnly rawRate)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(rawPosition);
         estimatedRate.set(rawRate);
         error.setToZero();
         biasEstimate.setToZero();
         return;
      }

      double kp = this.kp.getValue();
      double ki = this.ki.getValue();
      Tuple3DReadOnly x_meas = rawPosition;
      Tuple3DReadOnly xd_meas = rawRate;
      Vector3DBasics x_filt = this;
      xd_filt.add(xd_meas, biasEstimate);
      x_pred.scaleAdd(dt, xd_filt, x_filt);
      error.sub(x_meas, x_pred);
      x_filt.scaleAdd(kp, error, x_pred);
      biasEstimate.scaleAdd(ki, error, biasEstimate);
      estimatedRate.add(xd_meas, biasEstimate);
   }
}