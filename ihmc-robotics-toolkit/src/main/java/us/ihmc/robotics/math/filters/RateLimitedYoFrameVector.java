package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFrameTuple;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class RateLimitedYoFrameVector extends YoFrameVector
{
   private final YoDouble maxRateVariable;

   private final YoFrameTuple rawPosition;
   private final YoBoolean limited;
   private final YoBoolean hasBeenCalled;
   private final double dt;

   private final FrameVector3D differenceVector = new FrameVector3D();

   public RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                   YoFrameTuple rawPosition)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawPosition, rawPosition.getReferenceFrame());
   }

   public RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                   ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, null, referenceFrame);
   }

   public RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt, YoFrameTuple rawPosition)
   {
      this(namePrefix, nameSuffix, registry, null, dt, rawPosition, rawPosition.getReferenceFrame());
      setMaxRate(maxRate);
   }

   public RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, null, dt, null, referenceFrame);
      setMaxRate(maxRate);
   }

   private RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                    YoFrameTuple rawPosition, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.hasBeenCalled = new YoBoolean(namePrefix + "HasBeenCalled" + nameSuffix, registry);
      this.limited = new YoBoolean(namePrefix + "Limited" + nameSuffix, registry);

      if (maxRate != null)
         this.maxRateVariable = maxRate;
      else
         this.maxRateVariable = new YoDouble(namePrefix + "MaxRate" + nameSuffix, registry);

      this.rawPosition = rawPosition;

      this.dt = dt;

      reset();
   }

   public void setMaxRate(double maxRate)
   {
      this.maxRateVariable.set(maxRate);
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

      if (maxRateVariable.getDoubleValue() < 0)
         throw new RuntimeException("The maxRate parameter in the " + getClass().getSimpleName() + " cannot be negative.");

      differenceVector.setToZero(getReferenceFrame());
      differenceVector.set(xUnfiltered, yUnfiltered, zUnfiltered);
      differenceVector.sub(getX(), getY(), getZ());

      limited.set(differenceVector.clipToMaxLength(maxRateVariable.getDoubleValue() * dt));
      add(differenceVector);
   }
}
