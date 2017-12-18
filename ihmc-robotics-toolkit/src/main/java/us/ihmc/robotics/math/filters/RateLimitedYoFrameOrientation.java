package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameOrientation;

public class RateLimitedYoFrameOrientation extends YoFrameOrientation
{
   private final YoDouble maxRateVariable;

   private final YoFrameOrientation rawOrientation;
   private final YoBoolean limited;
   private final YoBoolean hasBeenCalled;
   private final double dt;

   private final FrameVector3D differenceVector = new FrameVector3D();

   public RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                   YoFrameOrientation rawOrientation)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawOrientation, rawOrientation.getReferenceFrame());
   }

   public RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                   ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, null, referenceFrame);
   }

   public RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt, YoFrameOrientation rawOrientation)
   {
      this(namePrefix, nameSuffix, registry, null, dt, rawOrientation, rawOrientation.getReferenceFrame());
      setMaxRate(maxRate);
   }

   public RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, null, dt, null, referenceFrame);
      setMaxRate(maxRate);
   }

   private RateLimitedYoFrameOrientation(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                    YoFrameOrientation rawOrientation, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.hasBeenCalled = new YoBoolean(namePrefix + "HasBeenCalled" + nameSuffix, registry);
      this.limited = new YoBoolean(namePrefix + "Limited" + nameSuffix, registry);

      if (maxRate != null)
         this.maxRateVariable = maxRate;
      else
         this.maxRateVariable = new YoDouble(namePrefix + "MaxRate" + nameSuffix, registry);

      this.rawOrientation = rawOrientation;

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
      if (rawOrientation == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(rawOrientation);
   }

   public void update(YoFrameOrientation yoFrameVectorUnfiltered)
   {
      checkReferenceFrameMatch(yoFrameVectorUnfiltered);
      update(yoFrameVectorUnfiltered.getYaw().getDoubleValue(), yoFrameVectorUnfiltered.getPitch().getDoubleValue(),
            yoFrameVectorUnfiltered.getRoll().getDoubleValue());
   }

   public void update(FrameQuaternion frameOrientationUnfiltered)
   {
      checkReferenceFrameMatch(frameOrientationUnfiltered);
      update(frameOrientationUnfiltered);
   }

   public void update(QuaternionReadOnly quaternionUnfiltered)
   {
      update(quaternionUnfiltered.getYaw(), quaternionUnfiltered.getPitch(), quaternionUnfiltered.getRoll());
   }

   public void update(double yawUnfiltered, double pitchUnfiltered, double rollUnfiltered)
   {
      if (!hasBeenCalled.getBooleanValue() || containsNaN())
      {
         hasBeenCalled.set(true);
         setYawPitchRoll(yawUnfiltered, pitchUnfiltered, rollUnfiltered);
      }

      if (maxRateVariable.getDoubleValue() < 0)
         throw new RuntimeException("The maxRate parameter in the " + getClass().getSimpleName() + " cannot be negative.");

      differenceVector.setToZero(getReferenceFrame());
      differenceVector.set(yawUnfiltered, pitchUnfiltered, rollUnfiltered);
      differenceVector.sub(getYaw().getDoubleValue(), getPitch().getDoubleValue(), getRoll().getDoubleValue());

      limited.set(differenceVector.clipToMaxLength(maxRateVariable.getDoubleValue() * dt));
      add(differenceVector.getX(), differenceVector.getY(), differenceVector.getZ());
   }
}
