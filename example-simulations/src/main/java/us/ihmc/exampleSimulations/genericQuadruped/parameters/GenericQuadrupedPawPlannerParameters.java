package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;

public class GenericQuadrupedPawPlannerParameters extends DefaultPawStepPlannerParameters
{

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepOutward()
   {
      return 0.25;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepInward()
   {
      return -0.05;
   }

   @Override
   public double getXGaitWeight()
   {
      return 1.0;
   }
}
