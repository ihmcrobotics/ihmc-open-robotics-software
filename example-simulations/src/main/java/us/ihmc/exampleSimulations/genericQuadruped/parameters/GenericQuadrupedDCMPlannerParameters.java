package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerParameters;

public class GenericQuadrupedDCMPlannerParameters implements DCMPlannerParameters
{

   /** {@inheritDoc} */
   @Override
   public double getSafeDistanceFromSupportPolygonEdges()
   {
      return 0.04;
   }

   /** {@inheritDoc} */
   @Override
   public double getStanceWidthCoPShiftFactor()
   {
      return 0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getStanceLengthCoPShiftFactor()
   {
      return -0.05;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxStanceWidthCoPShift()
   {
      return 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxStanceLengthCoPShift()
   {
      return 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepWidthCoPShiftFactor()
   {
      return 0.05;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthCoPShiftFactor()
   {
      return 0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxStepWidthCoPShift()
   {
      return 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxStepLengthCoPShift()
   {
      return 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumWeightShiftForward()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getAngleForMaxWeightShiftForward()
   {
      return Math.toRadians(20.0);
   }
}
