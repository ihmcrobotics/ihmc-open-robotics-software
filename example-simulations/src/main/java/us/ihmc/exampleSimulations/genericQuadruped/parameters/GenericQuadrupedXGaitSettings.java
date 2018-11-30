package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;

public class GenericQuadrupedXGaitSettings implements QuadrupedXGaitSettingsReadOnly
{
   @Override
   public double getStanceLength()
   {
      return 1.1;
   }

   @Override
   public double getStanceWidth()
   {
      return 0.2;
   }

   @Override
   public double getStepGroundClearance()
   {
      return 0.1;
   }

   @Override
   public double getStepDuration()
   {
      return 0.33;
   }

   @Override
   public double getEndDoubleSupportDuration()
   {
      return 0.1;
   }

   @Override
   public double getEndPhaseShift()
   {
      return 90.0;
   }
}
