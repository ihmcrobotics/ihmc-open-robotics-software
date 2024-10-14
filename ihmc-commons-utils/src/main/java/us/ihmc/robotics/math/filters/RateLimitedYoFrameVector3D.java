package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class RateLimitedYoFrameVector3D extends YoFrameVector3D
{
   private final DoubleProvider maxRateVariable;

   private final FrameTuple3DReadOnly rawPosition;
   private final YoBoolean limited;
   private final YoBoolean hasBeenCalled;
   private final double dt;

   private final FrameVector3D differenceVector = new FrameVector3D();


   public RateLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                     FrameTuple3DReadOnly rawPosition)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawPosition, rawPosition.getReferenceFrame());
   }

   public RateLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                     ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, null, referenceFrame);
   }

   public RateLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, double maxRate, double dt,
                                     FrameTuple3DReadOnly rawPosition)
   {
      this(namePrefix, nameSuffix, registry, VariableTools.createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, rawPosition,
           rawPosition.getReferenceFrame());
   }

   public RateLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, double maxRate, double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, VariableTools.createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, null, referenceFrame);
   }

   private RateLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                      FrameTuple3DReadOnly rawPosition, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(namePrefix, nameSuffix, registry);
      limited = VariableTools.createLimitedCalledYoBoolean(namePrefix, nameSuffix, registry);

      if (maxRate == null)
         maxRate = VariableTools.createMaxRateYoDouble(namePrefix, nameSuffix, Double.POSITIVE_INFINITY, registry);

      maxRateVariable = maxRate;

      this.rawPosition = rawPosition;

      this.dt = dt;

      reset();
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

   public void update(FrameTuple3DReadOnly frameVectorUnfiltered)
   {
      checkReferenceFrameMatch(frameVectorUnfiltered);
      update(frameVectorUnfiltered.getX(), frameVectorUnfiltered.getY(), frameVectorUnfiltered.getZ());
   }

   public void update(Tuple3DReadOnly vectorUnfiltered)
   {
      update(vectorUnfiltered.getX(), vectorUnfiltered.getY(), vectorUnfiltered.getZ());
   }

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      if (!hasBeenCalled.getBooleanValue() || containsNaN())
      {
         hasBeenCalled.set(true);
         set(xUnfiltered, yUnfiltered, zUnfiltered);
      }

      if (maxRateVariable.getValue() < 0)
         throw new RuntimeException("The maxRate parameter in the " + getClass().getSimpleName() + " cannot be negative.");

      differenceVector.setToZero(getReferenceFrame());
      differenceVector.set(xUnfiltered, yUnfiltered, zUnfiltered);
      differenceVector.sub(getX(), getY(), getZ());

      limited.set(differenceVector.clipToMaxNorm(maxRateVariable.getValue() * dt));
      add(differenceVector);
   }
}
