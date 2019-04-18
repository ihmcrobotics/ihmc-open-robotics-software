package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;

public class RateLimitedYoFrameQuaternion extends YoFrameQuaternion
{
   private final QuaternionReadOnly rawQuaternion;
   private final YoBoolean hasBeenCalled;
   private final double dt;

   private final YoBoolean limited;
   private final DoubleProvider maxRateVariable;

   private static DoubleProvider createMaxRateYoDouble(String namePrefix, String nameSuffix, double initialValue, YoVariableRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "MaxRate" + nameSuffix, registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   public RateLimitedYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt,
                                       FrameQuaternionReadOnly rawQuaternion)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, rawQuaternion.getReferenceFrame(),
           rawQuaternion);
   }

   public RateLimitedYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt,
                                       ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, referenceFrame, null);
   }

   public RateLimitedYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt,
                                       ReferenceFrame referenceFrame, QuaternionReadOnly rawQuaternion)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, referenceFrame, rawQuaternion);
   }

   public RateLimitedYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider maxRate, double dt,
                                       FrameQuaternionReadOnly rawQuaternion)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawQuaternion.getReferenceFrame(), rawQuaternion);
   }

   public RateLimitedYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider maxRate, double dt,
                                       ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, referenceFrame, null);
   }

   public RateLimitedYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider maxRate, double dt,
                                       ReferenceFrame referenceFrame, QuaternionReadOnly rawQuaternion)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      hasBeenCalled = new YoBoolean(namePrefix + "HasBeenCalled" + nameSuffix, registry);
      limited = new YoBoolean(namePrefix + "Limited" + nameSuffix, registry);

      if (maxRate == null)
         maxRate = createMaxRateYoDouble(namePrefix, nameSuffix, Double.POSITIVE_INFINITY, registry);

      maxRateVariable = maxRate;

      this.rawQuaternion = rawQuaternion;

      this.dt = dt;

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (rawQuaternion == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(rawQuaternion);
   }

   private final Quaternion quaternion = new Quaternion();

   public void update(FrameOrientation3DReadOnly frameOrientationUnfiltered)
   {
      checkReferenceFrameMatch(frameOrientationUnfiltered);
      quaternion.set(frameOrientationUnfiltered);
      update(quaternion);
   }

   public void update(FrameQuaternionReadOnly frameQuaternionUnfiltered)
   {
      checkReferenceFrameMatch(frameQuaternionUnfiltered);
      quaternion.set(frameQuaternionUnfiltered);
      update(quaternion);
   }

   private final Quaternion difference = new Quaternion();
   private final Vector3D limitedRotationVector = new Vector3D();

   public void update(QuaternionReadOnly quaternionUnfiltered)
   {
      if (!hasBeenCalled.getBooleanValue() || containsNaN())
      {
         hasBeenCalled.set(true);
         limited.set(false);
         set(quaternionUnfiltered);
         return;
      }

      if (dot(quaternionUnfiltered) > 0.0)
      {
         difference.difference(this, quaternionUnfiltered);
      }
      else
      {
         difference.setAndNegate(quaternionUnfiltered);
         difference.preMultiplyConjugateOther(this);
      }

      difference.getRotationVector(limitedRotationVector);
      boolean clipped = limitedRotationVector.clipToMaxLength(dt * maxRateVariable.getValue());
      limited.set(clipped);

      if (clipped)
      {
         difference.setRotationVector(limitedRotationVector);
         multiply(difference);
      }
      else
      {
         set(quaternionUnfiltered);
      }
   }
}
