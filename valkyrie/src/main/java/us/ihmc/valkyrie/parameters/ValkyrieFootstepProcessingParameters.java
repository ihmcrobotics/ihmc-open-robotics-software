package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepProcessingParameters;

public class ValkyrieFootstepProcessingParameters implements FootstepProcessingParameters
{
   @Override
   public double getMinimumSwingTime()
   {
      return 1.2;
   }

   @Override
   public double getMaximumSwingTime()
   {
      return 2.4;
   }

   @Override
   public double getMaximumStepTranslationForMinimumSwingTime()
   {
      return 0.2;
   }

   @Override
   public double getMinimumStepTranslationForMaximumSwingTime()
   {
      return 0.6;
   }
}
