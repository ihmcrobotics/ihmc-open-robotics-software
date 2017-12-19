package us.ihmc.footstepPlanning;

import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;

public class DefaultFootstepPlanningParameters implements FootstepPlannerParameters
{

   @Override
   public double getIdealFootstepWidth()
   {
      return 0.22;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return 0.3;
   }

   @Override
   public double getMaximumStepReach()
   {
      return 0.45;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return 0.4;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return 0.15;
   }

   @Override
   public double getMinimumStepLength()
   {
      return -0.1;
   }

   @Override
   public double getMaximumStepZ()
   {
      return 0.28;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return 0.4;
   }

   @Override
   public double getMaximumStepXWhenForwardAndDown()
   {
      return 0.35;
   }

   @Override
   public double getMaximumStepZWhenForwardAndDown()
   {
      return 0.10;
   }

   @Override
   public double getWiggleInsideDelta()
   {
      return 0.01;
   }
}
