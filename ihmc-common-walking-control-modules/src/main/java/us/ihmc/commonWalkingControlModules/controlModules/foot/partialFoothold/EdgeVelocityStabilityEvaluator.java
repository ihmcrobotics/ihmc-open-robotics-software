package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector2d;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class EdgeVelocityStabilityEvaluator
{
   /** Filtered data of the center of rotation linear velocity. */
   private final FilteredVelocityYoFrameVector2d centerOfRotationVelocity;
   /** Linear velocity of the center of rotation that is transverse (perpendicular) to the line of rotation. */
   private final YoDouble centerOfRotationTransverseVelocity;

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

   private final YoBoolean isEdgeStable;
   private final YoInteger stableCounter;

   private final FrameLine2DReadOnly lineOfRotation;

   public EdgeVelocityStabilityEvaluator(String namePrefix,
                                         FrameLine2DReadOnly lineOfRotation,
                                         DoubleProvider lineOfRotationStableVelocityThreshold,
                                         DoubleProvider centerOfRotationStableVelocityThreshold,
                                         IntegerProvider stableWindowWize,
                                         double dt,
                                         YoRegistry registry)
   {
      this.lineOfRotation = lineOfRotation;
      this.lineOfRotationStableVelocityThreshold = lineOfRotationStableVelocityThreshold;
      this.centerOfRotationStableVelocityThreshold = centerOfRotationStableVelocityThreshold;
      this.stableWindowWize = stableWindowWize;

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

      isEdgeStable = new YoBoolean(namePrefix + "IsEdgeStable", registry);
      stableCounter = new YoInteger(namePrefix + "IsEdgeStableCount", registry);
   }

   public void reset()
   {
      centerOfRotationVelocity.reset();
      centerOfRotationVelocity.setToNaN();

      angleOfLineOfRotation.set(0.0);
      lineOfRotationAngularVelocity.set(Double.NaN);
      lineOfRotationAngularVelocity.reset();
      stableCounter.set(0);

      isLineOfRotationStable.set(false);
      isCenterOfRotationStable.set(false);
      isEdgeStable.set(false);
   }

   public void update()
   {
      centerOfRotationVelocity.update();

      FrameVector2DReadOnly directionOfRotation = lineOfRotation.getDirection();
      centerOfRotationTransverseVelocity.set(centerOfRotationVelocity.cross(directionOfRotation) / directionOfRotation.length());

      angleOfLineOfRotation.set(Math.atan2(directionOfRotation.getY(), directionOfRotation.getX()));
      lineOfRotationAngularVelocity.updateForAngles();

      isLineOfRotationStable.set(Math.abs(lineOfRotationAngularVelocity.getDoubleValue()) < lineOfRotationStableVelocityThreshold.getValue());
      isCenterOfRotationStable.set(Math.abs(centerOfRotationTransverseVelocity.getDoubleValue()) < centerOfRotationStableVelocityThreshold.getValue());

      updateStableCount(isLineOfRotationStable.getBooleanValue(), isCenterOfRotationStable.getBooleanValue());
   }

   private void updateStableCount(boolean isLineOfRotationStable, boolean isCenterOfRotationStable)
   {
      boolean isStable = isLineOfRotationStable && isCenterOfRotationStable;
      
      if (isEdgeStable.getBooleanValue())
      {
         if (!isCenterOfRotationStable)
         {
            isEdgeStable.set(false);
            stableCounter.set(0);
            return;
         }
         else if (!isLineOfRotationStable)  
         {
            stableCounter.increment();
         }
         else
         {
            stableCounter.decrement();
            if (stableCounter.getIntegerValue() < 0)
               stableCounter.set(0);
         }
      }
      else
      {
         if (isStable)
         {
            stableCounter.increment();
         }
         else
         {
            stableCounter.decrement();
            if (stableCounter.getIntegerValue() < 0)
               stableCounter.set(0);
         }
      }

      if (stableCounter.getIntegerValue() > stableWindowWize.getValue() - 1)
      {
         isEdgeStable.set(isStable);
         stableCounter.set(0);
      }
   }

   public boolean isEdgeVelocityStable()
   {
      return isEdgeStable.getBooleanValue();
   }
}