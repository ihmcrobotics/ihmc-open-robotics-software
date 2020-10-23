package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RateLimitedYoFramePoint2D extends YoFramePoint2D
{
   private final DoubleProvider maxRateVariable;

   private final FrameTuple2DReadOnly rawPosition;
   private final YoBoolean limited;
   private final YoBoolean hasBeenCalled;
   private final double dt;

   private final FrameVector2D differenceVector = new FrameVector2D();

   private static DoubleProvider createMaxRateYoDouble(String namePrefix, String nameSuffix, double initialValue, YoRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "MaxRate" + nameSuffix, registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   public RateLimitedYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                  FrameTuple2DReadOnly rawPosition)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawPosition, rawPosition.getReferenceFrame());
   }

   public RateLimitedYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                  ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, null, referenceFrame);
   }

   public RateLimitedYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, double maxRate, double dt,
                                  FrameTuple2DReadOnly rawPosition)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, rawPosition,
           rawPosition.getReferenceFrame());
   }

   public RateLimitedYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, double maxRate, double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, null, referenceFrame);
   }

   private RateLimitedYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                   FrameTuple2DReadOnly rawPosition, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.hasBeenCalled = new YoBoolean(namePrefix + "HasBeenCalled" + nameSuffix, registry);
      this.limited = new YoBoolean(namePrefix + "Limited" + nameSuffix, registry);

      if (maxRate == null)
         maxRate = createMaxRateYoDouble(namePrefix, nameSuffix, Double.POSITIVE_INFINITY, registry);

      maxRateVariable = maxRate;

      this.rawPosition = rawPosition;

      this.dt = dt;

      reset();
   }

   public void setAndUpdate(FramePoint2DReadOnly framePoint2D)
   {
      super.set(framePoint2D);
      hasBeenCalled.set(true);
   }

   public void setAndUpdate(FramePoint3DReadOnly framePoint3D)
   {
      super.set(framePoint3D);
      hasBeenCalled.set(true);
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (rawPosition == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(rawPosition);
   }

   public void update(FrameTuple2DReadOnly frameVectorUnfiltered)
   {
      checkReferenceFrameMatch(frameVectorUnfiltered);
      update(frameVectorUnfiltered.getX(), frameVectorUnfiltered.getY());
   }

   public void update(Tuple2DReadOnly vectorUnfiltered)
   {
      update(vectorUnfiltered.getX(), vectorUnfiltered.getY());
   }

   public void update(double xUnfiltered, double yUnfiltered)
   {
      if (!hasBeenCalled.getBooleanValue() || containsNaN())
      {
         hasBeenCalled.set(true);
         set(xUnfiltered, yUnfiltered);
      }

      if (maxRateVariable.getValue() < 0)
         throw new RuntimeException("The maxRate parameter in the " + getClass().getSimpleName() + " cannot be negative.");

      differenceVector.setToZero(getReferenceFrame());
      differenceVector.set(xUnfiltered, yUnfiltered);
      differenceVector.sub(getX(), getY());

      limited.set(differenceVector.clipToMaxLength(maxRateVariable.getValue() * dt));
      add(differenceVector);
   }
}
