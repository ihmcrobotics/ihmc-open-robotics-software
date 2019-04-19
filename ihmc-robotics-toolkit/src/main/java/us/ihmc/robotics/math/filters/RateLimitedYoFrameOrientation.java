package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class RateLimitedYoFrameOrientation extends YoFrameYawPitchRoll
{
   private final DoubleProvider maxRateVariable;

   private final YoFrameYawPitchRoll rawOrientation;
   private final YoBoolean limited;
   private final YoBoolean hasBeenCalled;
   private final double dt;

   private static DoubleProvider createMaxRateYoDouble(String namePrefix, String nameSuffix, double initialValue, YoVariableRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "MaxRate" + nameSuffix, registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   public RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider maxRate, double dt,
                                        YoFrameYawPitchRoll rawOrientation)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawOrientation, rawOrientation.getReferenceFrame());
   }

   public RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider maxRate, double dt,
                                        ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, null, referenceFrame);
   }

   public RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt,
                                        YoFrameYawPitchRoll rawOrientation)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, rawOrientation,
           rawOrientation.getReferenceFrame());
   }

   public RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt,
                                        ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, null, referenceFrame);
   }

   private RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider maxRate, double dt,
                                         YoFrameYawPitchRoll rawOrientation, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.hasBeenCalled = new YoBoolean(namePrefix + "HasBeenCalled" + nameSuffix, registry);
      this.limited = new YoBoolean(namePrefix + "Limited" + nameSuffix, registry);

      if (maxRate == null)
         maxRate = createMaxRateYoDouble(namePrefix, nameSuffix, Double.POSITIVE_INFINITY, registry);

      maxRateVariable = maxRate;

      this.rawOrientation = rawOrientation;

      this.dt = dt;

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (rawOrientation == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(rawOrientation);
   }

   public void update(YoFrameYawPitchRoll yoFrameVectorUnfiltered)
   {
      checkReferenceFrameMatch(yoFrameVectorUnfiltered);
      update(yoFrameVectorUnfiltered.getYaw(), yoFrameVectorUnfiltered.getPitch(), yoFrameVectorUnfiltered.getRoll());
   }

   public void update(FrameQuaternion frameOrientationUnfiltered)
   {
      checkReferenceFrameMatch(frameOrientationUnfiltered);
      update((QuaternionReadOnly) frameOrientationUnfiltered);
   }

   private final Quaternion quaternionUnfiltered = new Quaternion();

   public void update(double yawUnfiltered, double pitchUnfiltered, double rollUnfiltered)
   {
      quaternionUnfiltered.setYawPitchRoll(yawUnfiltered, pitchUnfiltered, rollUnfiltered);
      update(quaternionUnfiltered);
   }

   private final Quaternion quaternionFiltered = new Quaternion();
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

      quaternionFiltered.set(this);

      if (quaternionFiltered.dot(quaternionUnfiltered) > 0.0)
      {
         difference.difference(quaternionFiltered, quaternionUnfiltered);
      }
      else
      {
         difference.setAndNegate(quaternionUnfiltered);
         difference.preMultiplyConjugateOther(quaternionFiltered);
      }

      difference.getRotationVector(limitedRotationVector);
      boolean clipped = limitedRotationVector.clipToMaxLength(dt * maxRateVariable.getValue());
      limited.set(clipped);

      if (clipped)
      {
         difference.setRotationVector(limitedRotationVector);
         quaternionFiltered.multiply(difference);
         set(quaternionFiltered);
      }
      else
      {
         set(quaternionUnfiltered);
      }
   }
}
