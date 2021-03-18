package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedRobotics.planning.WeightDistributionCalculator;

public interface DCMPlannerParameters
{
   /**
    * Returns the minimum distance inside the support polygon that the CoP waypoint must be located at. When the support polygon contains only two points,
    * the CoP waypoint is just guaranteed to be inside the support polygon, rather than inside by this distance.
    */
   double getSafeDistanceFromSupportPolygonEdges();

   /**
    * This variable is used to shift the CoP waypoint towards the center of the robot's body when in triple support. First, the average stance width is
    * calculated. Then, this value is used to multiply this stance width to determine the shift distance. When positive, this value shifts towards the side
    * with less support feet.
    */
   double getStanceWidthCoPShiftFactor();

   /**
    * This variable is used to shift the CoP waypoint towards the forward/aft when in triple support. First, the average stance length is
    * calculated. Then, this value is used to multiply this stance length to determine the shift distance. When positive, this value shifts towards the end
    * with less support feet.
    */
   double getStanceLengthCoPShiftFactor();

   /**
    * Returns the maximum absolute shift value that can be calculated from {@link #getStanceWidthCoPShiftFactor()}.
    */
   double getMaxStanceWidthCoPShift();

   /**
    * Returns the maximum absolute shift value that can be calculated from {@link #getStanceLengthCoPShiftFactor()}.
    */
   double getMaxStanceLengthCoPShift();

   /**
    * This variable is used to shift the CoP waypoint in the lateral direction of the upcoming steps when in triple of quadruple support. First, the average
    * width of the next steps is calculated. Then, this value is used to multiply this width to determine the shift distance.
    */
   double getStepWidthCoPShiftFactor();

   /**
    * This variable is used to shift the CoP waypoint in the forward direction of the upcoming steps when in triple of quadruple support. First, the average
    * length of the next steps is calculated. Then, this value is used to multiply this width to determine the shift distance.
    */
   double getStepLengthCoPShiftFactor();

   /**
    * Returns the maximum absolute shift value that can be calculated from {@link #getStepWidthCoPShiftFactor()}.
    */
   double getMaxStepWidthCoPShift();

   /**
    * Returns the maximum absolute shift value that can be calculated from {@link #getStepLengthCoPShiftFactor()}.
    */
   double getMaxStepLengthCoPShift();

   /**
    * Returns the maximum weight shift in the forward direction that can come from an angle.
    */
   double getMaximumWeightShiftForward();

   /**
    * Returns incline angle at which the weight should be shifted by {@link #getMaximumWeightShiftForward()}.
    */
   double getAngleForMaxWeightShiftForward();

   default WeightDistributionCalculator getWeightDistributionCalculatorForInclines()
   {
      return (pitchAngle ->
      {
         double percentTotal = MathTools.clamp(pitchAngle / getAngleForMaxWeightShiftForward(), 1.0);
         return percentTotal * getMaximumWeightShiftForward();
      });
   }


}
