package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class RateLimitedYoFrameVector extends YoFrameVector
{
   private final DoubleYoVariable maxRateVariable;

   private final YoFrameVector rawPosition;
   private final BooleanYoVariable limited;
   private final BooleanYoVariable hasBeenCalled;
   private final double dt;

   private final FrameVector differenceVector = new FrameVector();

   public RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable maxRate, double dt,
                                   YoFrameVector rawPosition)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawPosition, rawPosition.getReferenceFrame());
   }

   public RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable maxRate, double dt,
                                   ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, null, referenceFrame);
   }

   public RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt, YoFrameVector rawPosition)
   {
      this(namePrefix, nameSuffix, registry, null, dt, rawPosition, rawPosition.getReferenceFrame());
      setMaxRate(maxRate);
   }

   public RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, null, dt, null, referenceFrame);
      setMaxRate(maxRate);
   }

   private RateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable maxRate, double dt,
                                    YoFrameVector rawPosition, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.hasBeenCalled = new BooleanYoVariable(namePrefix + "HasBeenCalled" + nameSuffix, registry);
      this.limited = new BooleanYoVariable(namePrefix + "Limited" + nameSuffix, registry);

      if (maxRate != null)
         this.maxRateVariable = maxRate;
      else
         this.maxRateVariable = new DoubleYoVariable(namePrefix + "MaxRate" + nameSuffix, registry);

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

   public void update(YoFrameVector yoFrameVectorUnfiltered)
   {
      checkReferenceFrameMatch(yoFrameVectorUnfiltered);
      update(yoFrameVectorUnfiltered.getX(), yoFrameVectorUnfiltered.getY(), yoFrameVectorUnfiltered.getZ());
   }

   public void update(FrameVector frameVectorUnfiltered)
   {
      checkReferenceFrameMatch(frameVectorUnfiltered);
      update(frameVectorUnfiltered.getX(), frameVectorUnfiltered.getY(), frameVectorUnfiltered.getZ());
   }

   public void update(Vector3DReadOnly vectorUnfiltered)
   {
      update(vectorUnfiltered.getX(), vectorUnfiltered.getY(), vectorUnfiltered.getZ());
   }

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(xUnfiltered, yUnfiltered, zUnfiltered);
      }

      if (maxRateVariable.getDoubleValue() < 0)
         throw new RuntimeException("The maxRate parameter in the " + getClass().getSimpleName() + " cannot be negative.");

      differenceVector.setToZero(getReferenceFrame());
      differenceVector.set(xUnfiltered, yUnfiltered, zUnfiltered);
      differenceVector.sub(getX(), getY(), getZ());

      limited.set(differenceVector.limitLength(maxRateVariable.getDoubleValue() * dt));
      add(differenceVector);
   }
}
