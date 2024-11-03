package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.math.filters.IntegratorBiasCompensatorYoVariable.createKiYoDouble;
import static us.ihmc.robotics.math.filters.IntegratorBiasCompensatorYoVariable.createKpYoDouble;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
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
   private final YoFrameVector3D error;
   private final YoFrameVector3D estimatedRate;
   private final YoFrameVector3D estimatedRateBias;
   private final YoBoolean hasBeenCalled;

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   double kp,
                                                   double ki,
                                                   FrameTuple3DReadOnly rawPosition,
                                                   FrameTuple3DReadOnly rawRate,
                                                   double dt)
   {
      this(name, registry, kp, ki, rawPosition, rawRate, rawPosition.getReferenceFrame(), dt);
   }

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   double kp,
                                                   double ki,
                                                   FrameTuple3DReadOnly rawPosition,
                                                   FrameTuple3DReadOnly rawRate,
                                                   ReferenceFrame biasEstimationFrame,
                                                   double dt)
   {
      this(name, registry, createKpYoDouble(name, kp, registry), createKiYoDouble(name, ki, registry), rawPosition, rawRate, biasEstimationFrame, dt);
   }

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   DoubleProvider kp,
                                                   DoubleProvider ki,
                                                   ReferenceFrame referenceFrame,
                                                   double dt)
   {
      this(name, registry, kp, ki, referenceFrame, referenceFrame, dt);
   }

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   DoubleProvider kp,
                                                   DoubleProvider ki,
                                                   ReferenceFrame referenceFrame,
                                                   ReferenceFrame biasEstimationFrame,
                                                   double dt)
   {
      this(name, registry, kp, ki, null, null, referenceFrame, biasEstimationFrame, dt);
   }

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   DoubleProvider kp,
                                                   DoubleProvider ki,
                                                   FrameTuple3DReadOnly rawPosition,
                                                   FrameTuple3DReadOnly rawRate,
                                                   double dt)
   {
      this(name, registry, kp, ki, rawPosition, rawRate, rawPosition.getReferenceFrame(), dt);
   }

   public IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                   YoRegistry registry,
                                                   DoubleProvider kp,
                                                   DoubleProvider ki,
                                                   FrameTuple3DReadOnly rawPosition,
                                                   FrameTuple3DReadOnly rawRate,
                                                   ReferenceFrame biasEstimationFrame,
                                                   double dt)
   {
      this(name, registry, kp, ki, rawPosition, rawRate, rawPosition.getReferenceFrame(), biasEstimationFrame, dt);
   }

   private IntegratorBiasCompensatorYoFrameVector3D(String name,
                                                    YoRegistry registry,
                                                    DoubleProvider kp,
                                                    DoubleProvider ki,
                                                    FrameTuple3DReadOnly rawPosition,
                                                    FrameTuple3DReadOnly rawRate,
                                                    ReferenceFrame referenceFrame,
                                                    ReferenceFrame biasEstimationFrame,
                                                    double dt)
   {
      super(name, referenceFrame, registry);

      if (rawPosition != null && rawRate != null)
         rawPosition.checkReferenceFrameMatch(rawRate);

      this.kp = kp;
      this.ki = ki;
      this.dt = dt;
      this.rawPosition = rawPosition;
      this.rawRate = rawRate;
      this.error = new YoFrameVector3D(name + "PositionError", referenceFrame, registry);
      this.estimatedRate = new YoFrameVector3D(name + "EstimatedRate", referenceFrame, registry);
      this.estimatedRateBias = new YoFrameVector3D(name + "EstimatedRateBias", biasEstimationFrame, registry);
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

   public YoFrameVector3D getBiasEstimation()
   {
      return estimatedRateBias;
   }

   @Override
   public void update()
   {
      update(rawPosition, rawRate);
   }

   private final Vector3D xd_filt = new Vector3D();
   private final Vector3D x_pred = new Vector3D();
   private final FrameVector3D intermediateBias = new FrameVector3D();

   public void update(FrameTuple3DReadOnly rawPosition, FrameTuple3DReadOnly rawRate)
   {
      if (rawPosition != null)
         checkReferenceFrameMatch(rawPosition);
      checkReferenceFrameMatch(rawRate);
      update((Tuple3DReadOnly) rawPosition, (Tuple3DReadOnly) rawRate);
   }

   public void update(Tuple3DReadOnly rawPosition, Tuple3DReadOnly rawRate)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         if (rawPosition != null)
         {
            hasBeenCalled.set(true);
            set(rawPosition);
         }
         estimatedRate.set(rawRate);
         error.setToZero();
         estimatedRateBias.setToZero();
         return;
      }

      intermediateBias.setIncludingFrame(estimatedRateBias);
      intermediateBias.changeFrame(getReferenceFrame());

      Vector3DBasics x_filt = this;
      xd_filt.add(rawRate, intermediateBias); // = xd_filt_new
      xd_filt.interpolate(estimatedRate, 0.5); // = 0.5 * (xd_filt_new + xd_filt_old)
      x_pred.scaleAdd(dt, xd_filt, x_filt);
      if (rawPosition != null)
      {
         error.sub(rawPosition, x_pred);
         x_filt.scaleAdd(kp.getValue(), error, x_pred);
         intermediateBias.scaleAdd(ki.getValue(), error, intermediateBias);
      }
      else
      { // No position update, using the prediction
         x_filt.set(x_pred);
      }
      estimatedRate.add(rawRate, intermediateBias);
      estimatedRateBias.setMatchingFrame(intermediateBias);
   }
}