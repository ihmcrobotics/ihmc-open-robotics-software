package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class GenericQuadrupedFootstepPlannerParameters extends DefaultFootstepPlannerParameters
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
