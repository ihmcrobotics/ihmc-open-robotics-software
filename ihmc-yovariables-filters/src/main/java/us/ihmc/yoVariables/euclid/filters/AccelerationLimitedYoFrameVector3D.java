package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.VariableTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class AccelerationLimitedYoFrameVector3D extends YoFrameVector3D
{
   private final DoubleProvider maxRateVariable;
   private final DoubleProvider maxAccelerationVariable;

   private final FrameTuple3DReadOnly rawPosition;
   private final YoBoolean accelerationLimited;
   private final YoBoolean rateLimited;

   private final YoBoolean hasBeenInitialized;
   private final YoDouble positionGain;
   private final YoDouble velocityGain;

   private final YoFrameVector3D smoothedRate;
   private final YoFrameVector3D smoothedAcceleration;

   private final double dt;

   private final FrameVector3D positionError;
   private final FrameVector3D acceleration;

   public AccelerationLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate,
                                             DoubleProvider maxAcceleration, double dt, FrameTuple3DReadOnly rawPosition)
   {
      this(namePrefix, nameSuffix, registry, maxRate, maxAcceleration, dt, rawPosition, rawPosition.getReferenceFrame());
   }

   public AccelerationLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate,
                                             DoubleProvider maxAcceleration, double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, maxAcceleration, dt, null, referenceFrame);
   }
   public AccelerationLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, double maxRate, double maxAcceleration,
                                             double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, VariableTools.createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry),
           VariableTools.createMaxAccelerationYoDouble(namePrefix, nameSuffix, maxAcceleration, registry), dt, null, referenceFrame);
   }

   private AccelerationLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate,
                                              DoubleProvider maxAcceleration, double dt, FrameTuple3DReadOnly rawPosition, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      positionError = new FrameVector3D(referenceFrame);
      acceleration = new FrameVector3D(referenceFrame);

      if (maxRate == null)
         maxRate = VariableTools.createMaxRateYoDouble(namePrefix, nameSuffix, Double.POSITIVE_INFINITY, registry);
      if (maxAcceleration == null)
         maxAcceleration = VariableTools.createMaxAccelerationYoDouble(namePrefix, nameSuffix, Double.POSITIVE_INFINITY, registry);

      this.maxRateVariable = maxRate;
      this.maxAccelerationVariable = maxAcceleration;

      this.dt = dt;

      hasBeenInitialized = new YoBoolean(namePrefix + "HasBeenInitialized" + namePrefix, registry);
      this.rateLimited = new YoBoolean(namePrefix + "RateLimited" + nameSuffix, registry);
      this.accelerationLimited = new YoBoolean(namePrefix + "AccelerationLimited" + nameSuffix, registry);

      smoothedRate = new YoFrameVector3D(namePrefix + "SmoothedRate" + namePrefix, referenceFrame, registry);
      smoothedAcceleration = new YoFrameVector3D(namePrefix + "SmoothedAcceleration" + namePrefix, referenceFrame, registry);

      positionGain = new YoDouble(namePrefix + "PositionGain" + namePrefix, registry);
      velocityGain = new YoDouble(namePrefix + "VelocityGain" + namePrefix, registry);

      double w0 = 2.0 * Math.PI * 16.0;
      double zeta = 1.0;

      setGainsByPolePlacement(w0, zeta);
      hasBeenInitialized.set(false);

      this.rawPosition = rawPosition;

   }


   public void setGainsByPolePlacement(double w0, double zeta)
   {
      positionGain.set(w0 * w0);
      velocityGain.set(2.0 * zeta * w0);
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

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      if (!hasBeenInitialized.getBooleanValue())
         initialize(xUnfiltered, yUnfiltered, zUnfiltered);

      positionError.set(xUnfiltered, yUnfiltered, zUnfiltered);
      positionError.sub(this);

      acceleration.set(smoothedRate);
      acceleration.scale(-velocityGain.getValue());
      acceleration.scaleAdd(positionGain.getValue(), positionError, acceleration);

      accelerationLimited.set(acceleration.clipToMaxLength(maxAccelerationVariable.getValue()));

      smoothedAcceleration.set(acceleration);
      smoothedRate.scaleAdd(dt, smoothedAcceleration, smoothedRate);

      rateLimited.set(smoothedRate.clipToMaxLength(maxRateVariable.getValue()));

      this.scaleAdd(dt, smoothedRate, this);

      if (this.containsNaN())
         throw new RuntimeException("what?");

   }

   public void initialize(FrameTuple3DReadOnly input)
   {
      initialize(input.getX(), input.getY(), input.getZ());
   }

   public void initialize(double xInput, double yInput, double zInput)
   {
      this.set(xInput, yInput, zInput);
      smoothedRate.setToZero();
      smoothedAcceleration.setToZero();

      this.hasBeenInitialized.set(true);
   }

   public void reset()
   {
      this.hasBeenInitialized.set(false);
      smoothedRate.setToZero();
      smoothedAcceleration.setToZero();
   }
}
