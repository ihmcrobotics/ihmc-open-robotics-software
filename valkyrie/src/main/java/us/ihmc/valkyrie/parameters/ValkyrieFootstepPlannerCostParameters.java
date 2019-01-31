package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class ValkyrieFootstepPlannerCostParameters implements FootstepPlannerCostParameters
{
   @Override
   public DoubleProvider getAStarHeuristicsWeight()
   {
      return () -> 5.0;
   }

   @Override
   public double getYawWeight()
   {
      return 0.15;
   }

   @Override
   public boolean useQuadraticDistanceCost()
   {
      return true;
   }

   @Override
   public double getForwardWeight()
   {
      return 2.5;
   }

   @Override
   public double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return 0.15;
   }

   @Override
   public double getBoundingBoxCost()
   {
      return 40.0;
   }
}
