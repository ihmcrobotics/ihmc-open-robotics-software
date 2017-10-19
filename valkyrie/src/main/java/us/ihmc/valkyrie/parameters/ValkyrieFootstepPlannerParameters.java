package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;

public class ValkyrieFootstepPlannerParameters implements FootstepPlannerParameters
{

   @Override
   public double getIdealFootstepWidth()
   {
      return 0.2;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return 0.2;
   }

   @Override
   public double getMaximumStepReach()
   {
      return 0.4;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return 0.6;
   }

   @Override
   public double getMinimumStepYaw()
   {
      return -0.15;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return 0.15;
   }

   @Override
   public double getMaximumStepZ()
   {
      return 0.1;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return 0.4;
   }
}
