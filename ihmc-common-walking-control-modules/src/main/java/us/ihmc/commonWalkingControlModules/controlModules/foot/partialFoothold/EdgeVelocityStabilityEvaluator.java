package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector2d;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class EdgeVelocityStabilityEvaluator
{
   /** Filtered data of the center of rotation linear velocity. */
   private final FilteredVelocityYoFrameVector2d centerOfRotationVelocity;
   /** Linear velocity of the center of rotation that is transverse (perpendicular) to the line of rotation. */
   private final YoDouble centerOfRotationTransverseVelocity;

   private final YoInteger numberOfTicksInEstimate;
   private final IntegerProvider minimumTicksToEstimate;
   private final IntegerProvider stableWindowWize;

   /** Absolute angle of the line of rotation. */
   private final YoDouble angleOfLineOfRotation;
   /** Filtered yaw rate of the line of rotation. */
   private final FilteredVelocityYoVariable lineOfRotationAngularVelocity;

   /** Threshold on the transverse velocity of the CoR w.r.t. the LoR to determine whether or not the CoR is stable. */
   private final DoubleProvider centerOfRotationStableVelocityThreshold;
   private final YoBoolean isCenterOfRotationStable;

   /** Threshold on the yaw rate of the line of rotation to determine whether or not the line of rotation is stable. */
   private final DoubleProvider lineOfRotationStableVelocityThreshold;
   private final YoBoolean isLineOfRotationStable;

   private final YoBoolean isEdgeVelocityStable;
   private final GlitchFilteredYoBoolean isEdgeStable;

   private final FrameLine2DReadOnly lineOfRotation;

   public EdgeVelocityStabilityEvaluator(String namePrefix,
                                         FrameLine2DReadOnly lineOfRotation,
                                         DoubleProvider lineOfRotationStableVelocityThreshold,
                                         DoubleProvider centerOfRotationStableVelocityThreshold,
                                         IntegerProvider minimumTicksToEstimate,
                                         IntegerProvider stableWindowWize,
                                         double dt,
                                         YoVariableRegistry registry)
   {
      this.lineOfRotation = lineOfRotation;
      this.lineOfRotationStableVelocityThreshold = lineOfRotationStableVelocityThreshold;
      this.centerOfRotationStableVelocityThreshold = centerOfRotationStableVelocityThreshold;
      this.minimumTicksToEstimate = minimumTicksToEstimate;
      this.stableWindowWize = stableWindowWize;

      numberOfTicksInEstimate = new YoInteger(namePrefix + "NumberOfTicksInEstimate", registry);

      YoDouble centerOfRotationVelocityAlphaFilter = new YoDouble(namePrefix + "CenterOfRotationVelocityAlphaFilter", registry);
      centerOfRotationVelocity = new FilteredVelocityYoFrameVector2d(namePrefix + "CenterOfRotationVelocity",
                                                                     "",
                                                                     centerOfRotationVelocityAlphaFilter,
                                                                     dt,
                                                                     registry,
                                                                     lineOfRotation.getPoint());
      centerOfRotationTransverseVelocity = new YoDouble(namePrefix + "CenterOfRotationTransverseVelocity", registry);

      angleOfLineOfRotation = new YoDouble(namePrefix + "AngleOfLineOfRotation", registry);
      YoDouble lineOfRotationAngularVelocityAlphaFilter = new YoDouble(namePrefix + "LineOfRotationAngularVelocityAlphaFilter", registry);
      lineOfRotationAngularVelocity = new FilteredVelocityYoVariable(namePrefix + "LineOfRotationAngularVelocityFiltered",
                                                                     "",
                                                                     lineOfRotationAngularVelocityAlphaFilter,
                                                                     angleOfLineOfRotation,
                                                                     dt,
                                                                     registry);

      isLineOfRotationStable = new YoBoolean(namePrefix + "IsLineOfRotationStable", registry);
      isCenterOfRotationStable = new YoBoolean(namePrefix + "IsCenterOfRotationStable", registry);
      isEdgeVelocityStable = new YoBoolean(namePrefix + "IsEdgeVelocityStable", registry);

      isEdgeStable = new GlitchFilteredYoBoolean(namePrefix + "IsEdgeStable", registry, isEdgeVelocityStable, 10);
   }

   public void reset()
   {
      centerOfRotationVelocity.reset();
      centerOfRotationVelocity.setToNaN();

      numberOfTicksInEstimate.set(0);
      angleOfLineOfRotation.set(0.0);
      lineOfRotationAngularVelocity.set(Double.NaN);
      lineOfRotationAngularVelocity.reset();

      isLineOfRotationStable.set(false);
      isCenterOfRotationStable.set(false);
      isEdgeVelocityStable.set(false);
      isEdgeStable.set(false);
   }

   public void update()
   {
      isEdgeStable.setWindowSize(stableWindowWize.getValue());
      numberOfTicksInEstimate.increment();
      centerOfRotationVelocity.update();

      FrameVector2DReadOnly directionOfRotation = lineOfRotation.getDirection();
      centerOfRotationTransverseVelocity.set(centerOfRotationVelocity.cross(directionOfRotation) / directionOfRotation.length());

      angleOfLineOfRotation.set(Math.atan2(directionOfRotation.getY(), directionOfRotation.getX()));
      lineOfRotationAngularVelocity.updateForAngles();

      isLineOfRotationStable.set(Math.abs(lineOfRotationAngularVelocity.getDoubleValue()) < lineOfRotationStableVelocityThreshold.getValue());
      isCenterOfRotationStable.set(Math.abs(centerOfRotationTransverseVelocity.getDoubleValue()) < centerOfRotationStableVelocityThreshold.getValue());

      if (numberOfTicksInEstimate.getIntegerValue() > minimumTicksToEstimate.getValue())
         isEdgeVelocityStable.set(isLineOfRotationStable.getBooleanValue() && isCenterOfRotationStable.getBooleanValue());
      else
         isEdgeVelocityStable.set(false);

      if (!isEdgeVelocityStable.getBooleanValue())
         isEdgeStable.set(false);
      else
         isEdgeStable.update();
   }

   public boolean isEdgeVelocityStable()
   {
      return isEdgeStable.getBooleanValue();
   }
}