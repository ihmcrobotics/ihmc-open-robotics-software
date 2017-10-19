package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;

public class AtlasFootstepPlannerParameters implements FootstepPlannerParameters
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
   public double getMaximumStepZ()
   {
      return 0.25;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return 0.4;
   }

}
