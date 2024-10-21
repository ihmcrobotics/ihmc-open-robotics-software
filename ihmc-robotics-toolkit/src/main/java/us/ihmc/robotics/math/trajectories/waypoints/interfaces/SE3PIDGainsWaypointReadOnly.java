package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;

import java.util.Arrays;

public interface SE3PIDGainsWaypointReadOnly
{
   PID3DGains getAngular();

   PID3DGains getLinear();

   default double[] getAngularProportionalGains()
   {
      return getAngular().getProportionalGains();
   }

   default double[] getAngularIntegralGains()
   {
      return getAngular().getIntegralGains();
   }

   default double[] getAngularDerivativeGains()
   {
      return getAngular().getDerivativeGains();
   }

   default double[] getAngularDampingRatios()
   {
      return getAngular().getDampingRatios();
   }

   default double getAngularMaximumIntegralError()
   {
      return getAngular().getMaximumIntegralError();
   }

   default double getAngularMaximumDerivativeError()
   {
      return getAngular().getMaximumDerivativeError();
   }

   default double getAngularMaximumProportionalError()
   {
      return getAngular().getMaximumProportionalError();
   }

   default double getAngularMaximumFeedback()
   {
      return getAngular().getMaximumFeedback();
   }

   default double getAngularMaximumFeedbackRate()
   {
      return getAngular().getMaximumFeedbackRate();
   }

   default double[] getLinearProportionalGains()
   {
      return getLinear().getProportionalGains();
   }

   default double[] getLinearIntegralGains()
   {
      return getLinear().getIntegralGains();
   }

   default double[] getLinearDerivativeGains()
   {
      return getLinear().getDerivativeGains();
   }

   default double[] getLinearDampingRatios()
   {
      return getLinear().getDampingRatios();
   }

   default double getLinearMaximumIntegralError()
   {
      return getLinear().getMaximumIntegralError();
   }

   default double getLinearMaximumDerivativeError()
   {
      return getLinear().getMaximumDerivativeError();
   }

   default double getLinearMaximumProportionalError()
   {
      return getLinear().getMaximumProportionalError();
   }

   default double getLinearMaximumFeedback()
   {
      return getLinear().getMaximumFeedback();
   }

   default double getLinearMaximumFeedbackRate()
   {
      return getLinear().getMaximumFeedbackRate();
   }

   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SE3PIDGainsWaypointReadOnly))
         return false;

      SE3PIDGainsWaypointReadOnly other = (SE3PIDGainsWaypointReadOnly) geometry;

      if (!getAngular().equals(other.getAngular()))
         return false;
      if (!getLinear().equals(other.getLinear()))
         return false;
      return true;
   }

   default boolean epsilonEquals(SE3PIDGainsWaypointReadOnly other, double epsilon)
   {
//      PIDGains does not have an epsilonEquals method
      if (!getAngular().equals(other.getAngular()))
         return false;
      if (!getLinear().equals(other.getLinear()))
         return false;
      return true;
   }

   default boolean containsNaN()
   {
      return Arrays.stream(getAngularProportionalGains()).anyMatch(Double::isNaN);
   }

   default String toString(String format)
   {
      return String.format("Angular: %s, Linear: %s", getAngular().toString(), getLinear().toString());
   }
}