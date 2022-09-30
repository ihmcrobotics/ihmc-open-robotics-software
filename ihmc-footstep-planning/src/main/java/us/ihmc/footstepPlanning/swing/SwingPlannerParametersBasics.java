package us.ihmc.footstepPlanning.swing;

import toolbox_msgs.msg.dds.SwingPlannerParametersPacket;
import us.ihmc.euclid.Axis3D;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface SwingPlannerParametersBasics extends SwingPlannerParametersReadOnly, StoredPropertySetBasics
{
   default void set(SwingPlannerParametersReadOnly swingPlannerParameters)
   {
      setAll(swingPlannerParameters.getAll());
   }

   default void setSwingHeightIfCollisionDetected(double swingHeightIfCollisionDetected)
   {
      set(SwingPlannerParameterKeys.swingHeightIfCollisionDetected, swingHeightIfCollisionDetected);
   }

   default void setMinimumSwingTime(double minimumSwingTime)
   {
      set(SwingPlannerParameterKeys.minimumSwingTime, minimumSwingTime);
   }

   default void setMaximumSwingTime(double maximumSwingTime)
   {
      set(SwingPlannerParameterKeys.maximumSwingTime, maximumSwingTime);
   }

   default void setFootStubClearance(double footStubClearance)
   {
      set(SwingPlannerParameterKeys.footStubClearance, footStubClearance);
   }

   default void setWaypointProportionShiftForStubAvoidance(double waypointProportionShiftForStubAvoidance)
   {
      set(SwingPlannerParameterKeys.waypointProportionShiftForStubAvoidance, waypointProportionShiftForStubAvoidance);
   }

   default void setDoInitialFastApproximation(boolean doInitialFastApproximation)
   {
      set(SwingPlannerParameterKeys.doInitialFastApproximation, doInitialFastApproximation);
   }

   default void setMinimumSwingFootClearance(double minimumSwingFootClearance)
   {
      set(SwingPlannerParameterKeys.minimumSwingFootClearance, minimumSwingFootClearance);
   }

   default void setFastApproximationLessClearance(double fastApproximationLessClearance)
   {
      set(SwingPlannerParameterKeys.fastApproximationLessClearance, fastApproximationLessClearance);
   }

   default void setNumberOfChecksPerSwing(int numberOfChecksPerSwing)
   {
      set(SwingPlannerParameterKeys.numberOfChecksPerSwing, numberOfChecksPerSwing);
   }

   default void setMaximumNumberOfAdjustmentAttempts(int maximumNumberOfAdjustmentAttempts)
   {
      set(SwingPlannerParameterKeys.maximumNumberOfAdjustmentAttempts, maximumNumberOfAdjustmentAttempts);
   }

   default void setMaximumWaypointAdjustmentDistance(double maximumWaypointAdjustmentDistance)
   {
      set(SwingPlannerParameterKeys.maximumWaypointAdjustmentDistance, maximumWaypointAdjustmentDistance);
   }

   default void setMinimumAdjustmentIncrementDistance(double minimumAdjustmentIncrementDistance)
   {
      set(SwingPlannerParameterKeys.minimumAdjustmentIncrementDistance, minimumAdjustmentIncrementDistance);
   }

   default void setMaximumAdjustmentIncrementDistance(double maximumAdjustmentIncrementDistance)
   {
      set(SwingPlannerParameterKeys.maximumAdjustmentIncrementDistance, maximumAdjustmentIncrementDistance);
   }

   default void setAdjustmentIncrementDistanceGain(double adjustmentIncrementDistanceGain)
   {
      set(SwingPlannerParameterKeys.adjustmentIncrementDistanceGain, adjustmentIncrementDistanceGain);
   }

   default void setMinimumHeightAboveFloorForCollision(double minimumHeightAboveFloorForCollision)
   {
      set(SwingPlannerParameterKeys.minimumHeightAboveFloorForCollision, minimumHeightAboveFloorForCollision);
   }

   default void setAdditionalSwingTimeIfExpanded(double additionalSwingTimeIfExpanded)
   {
      set(SwingPlannerParameterKeys.additionalSwingTimeIfExpanded, additionalSwingTimeIfExpanded);
   }

   default void setAllowLateralMotion(boolean allowLateralMotion)
   {
      set(SwingPlannerParameterKeys.allowLateralMotion, allowLateralMotion);
   }

   default void setExtraSizePercentageLow(Axis3D axis, double extraSizePercentageLow)
   {
      switch (axis)
      {
         case X:
            set(SwingPlannerParameterKeys.percentageExtraSizeXLow, extraSizePercentageLow);
            break;
         case Y:
            set(SwingPlannerParameterKeys.percentageExtraSizeYLow, extraSizePercentageLow);
            break;
         case Z:
            set(SwingPlannerParameterKeys.percentageExtraSizeZLow, extraSizePercentageLow);
            break;
         default:
            throw new RuntimeException("Unknown axis " + axis);
      }
   }

   default void setExtraSizePercentageHigh(Axis3D axis, double extraSizePercentageLow)
   {
      switch (axis)
      {
         case X:
            set(SwingPlannerParameterKeys.percentageExtraSizeXHigh, extraSizePercentageLow);
            break;
         case Y:
            set(SwingPlannerParameterKeys.percentageExtraSizeYHigh, extraSizePercentageLow);
            break;
         case Z:
            set(SwingPlannerParameterKeys.percentageExtraSizeZHigh, extraSizePercentageLow);
            break;
         default:
            throw new RuntimeException("Unknown axis " + axis);
      }
   }

   default void setExtraSizeLow(Axis3D axis, double extraSizeLow)
   {
      switch (axis)
      {
         case X:
            set(SwingPlannerParameterKeys.extraSizeXLow, extraSizeLow);
            break;
         case Y:
            set(SwingPlannerParameterKeys.extraSizeYLow, extraSizeLow);
            break;
         case Z:
            set(SwingPlannerParameterKeys.extraSizeZLow, extraSizeLow);
            break;
         default:
            throw new RuntimeException("Unknown axis " + axis);
      }
   }

   default void setExtraSizeHigh(Axis3D axis, double extraSizeHigh)
   {
      switch (axis)
      {
         case X:
            set(SwingPlannerParameterKeys.extraSizeXHigh, extraSizeHigh);
            break;
         case Y:
            set(SwingPlannerParameterKeys.extraSizeYHigh, extraSizeHigh);
            break;
         case Z:
            set(SwingPlannerParameterKeys.extraSizeZHigh, extraSizeHigh);
            break;
         default:
            throw new RuntimeException("Unknown axis " + axis);
      }
   }

   default void setPercentageLowMaxDisplacement(double percentageLowMaxDisplacement)
   {
      set(SwingPlannerParameterKeys.percentageMaxDisplacementLow, percentageLowMaxDisplacement);
   }

   default void setPercentageHighMaxDisplacement(double percentageHighMaxDisplacement)
   {
      set(SwingPlannerParameterKeys.percentageMaxDisplacementHigh, percentageHighMaxDisplacement);
   }

   default void setMaxDisplacementLow(double maxDisplacementLow)
   {
      set(SwingPlannerParameterKeys.maxDisplacementLow, maxDisplacementLow);
   }

   default void setMaxDisplacementHigh(double maxDisplacementHigh)
   {
      set(SwingPlannerParameterKeys.maxDisplacementHigh, maxDisplacementHigh);
   }

   default void setMotionCorrelationAlpha(double motionCorrelationAlpha)
   {
      set(SwingPlannerParameterKeys.motionCorrelationAlpha, motionCorrelationAlpha);
   }

   default void setMinXYTranslationToPlanSwing(double minXYTranslationToPlanSwing)
   {
      set(SwingPlannerParameterKeys.minXYTranslationToPlanSwing, minXYTranslationToPlanSwing);
   }

   default void set(SwingPlannerParametersPacket packet)
   {
      setDoInitialFastApproximation(packet.getDoInitialFastApproximation());
      setNumberOfChecksPerSwing((int) packet.getNumberOfChecksPerSwing());
      setMaximumNumberOfAdjustmentAttempts((int) packet.getMaximumNumberOfAdjustmentAttempts());

      if (packet.getSwingHeightIfCollisionDetected() != -1.0)
         setSwingHeightIfCollisionDetected(packet.getSwingHeightIfCollisionDetected());
      if (packet.getMinimumSwingTime() != -1.0)
         setMinimumSwingTime(packet.getMinimumSwingTime());
      if (packet.getMaximumSwingTime() != -1.0)
         setMaximumSwingTime(packet.getMaximumSwingTime());
      if (packet.getFootStubClearance() != -1.0)
         setFootStubClearance(packet.getFootStubClearance());
      if (packet.getWaypointProportionShiftForStubAvoidance() != -1.0)
         setWaypointProportionShiftForStubAvoidance(packet.getWaypointProportionShiftForStubAvoidance());

      if (packet.getMinimumSwingFootClearance() != -1.0)
         setMinimumSwingFootClearance(packet.getMinimumSwingFootClearance());
      if (packet.getFastApproximationLessClearance() != -1.0)
         setFastApproximationLessClearance(packet.getFastApproximationLessClearance());
      if (packet.getMaximumWaypointAdjustmentDistance() != -1.0)
         setMaximumWaypointAdjustmentDistance(packet.getMaximumWaypointAdjustmentDistance());
      if (packet.getMinimumAdjustmentIncrementDistance() != -1.0)
         setMinimumAdjustmentIncrementDistance(packet.getMinimumAdjustmentIncrementDistance());
      if (packet.getMaximumAdjustmentIncrementDistance() != -1.0)
         setMaximumAdjustmentIncrementDistance(packet.getMaximumAdjustmentIncrementDistance());
      if (packet.getAdjustmentIncrementDistanceGain() != -1.0)
         setAdjustmentIncrementDistanceGain(packet.getAdjustmentIncrementDistanceGain());
      if (packet.getMinimumHeightAboveFloorForCollision() != -1.0)
         setMinimumHeightAboveFloorForCollision(packet.getMinimumHeightAboveFloorForCollision());
      if (packet.getAdditionalSwingTimeIfExpanded() != -1.0)
         setAdditionalSwingTimeIfExpanded(packet.getAdditionalSwingTimeIfExpanded());

      if (packet.getPercentageExtraSizeXLow() != -1.0)
         setExtraSizePercentageLow(Axis3D.X, packet.getPercentageExtraSizeXLow());
      if (packet.getPercentageExtraSizeXHigh() != -1.0)
         setExtraSizePercentageHigh(Axis3D.X, packet.getPercentageExtraSizeXHigh());
      if (packet.getExtraSizeXLow() != -1.0)
         setExtraSizeLow(Axis3D.X, packet.getExtraSizeXLow());
      if (packet.getExtraSizeXHigh() != -1.0)
         setExtraSizeHigh(Axis3D.X, packet.getExtraSizeXHigh());
      if (packet.getPercentageExtraSizeYLow() != -1.0)
         setExtraSizePercentageLow(Axis3D.Y, packet.getPercentageExtraSizeYLow());
      if (packet.getPercentageExtraSizeYHigh() != -1.0)
         setExtraSizePercentageHigh(Axis3D.Y, packet.getPercentageExtraSizeYHigh());
      if (packet.getExtraSizeYLow() != -1.0)
         setExtraSizeLow(Axis3D.Y, packet.getExtraSizeYLow());
      if (packet.getExtraSizeYHigh() != -1.0)
         setExtraSizeHigh(Axis3D.Y, packet.getExtraSizeYHigh());
      if (packet.getPercentageExtraSizeZLow() != -1.0)
         setExtraSizePercentageLow(Axis3D.Z, packet.getPercentageExtraSizeZLow());
      if (packet.getPercentageExtraSizeZHigh() != -1.0)
         setExtraSizePercentageHigh(Axis3D.Z, packet.getPercentageExtraSizeZHigh());
      if (packet.getExtraSizeZLow() != -1.0)
         setExtraSizeLow(Axis3D.Z, packet.getExtraSizeZLow());
      if (packet.getExtraSizeZHigh() != -1.0)
         setExtraSizeHigh(Axis3D.Z, packet.getExtraSizeZHigh());

      setPercentageLowMaxDisplacement(packet.getPercentageMaxDisplacementLow());
      setPercentageHighMaxDisplacement(packet.getPercentageMaxDisplacementHigh());
      setMaxDisplacementLow(packet.getMaxDisplacementLow());
      setMaxDisplacementHigh(packet.getMaxDisplacementHigh());
      setMotionCorrelationAlpha(packet.getMotionCorrelationAlpha());
      setAllowLateralMotion(packet.getAllowLateralMotion());

      if (packet.getMinXyTranslationToPlanSwing() != -1.0)
         setMinXYTranslationToPlanSwing(packet.getMinXyTranslationToPlanSwing());
   }
}
