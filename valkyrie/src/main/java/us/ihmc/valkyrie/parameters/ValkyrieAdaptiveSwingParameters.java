package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;

public class ValkyrieAdaptiveSwingParameters implements AdaptiveSwingParameters
{
   /**
    * Parameters for setting swing height as a function of vertical step displacement
    */

   @Override
   public double getMinimumSwingHeight()
   {
      return 0.03;
   }

   @Override
   public double getMaximumSwingHeight()
   {
      return 0.08;
   }

   @Override
   public double getMaximumStepHeightForMinimumSwingHeight()
   {
      return 0.02;
   }

   @Override
   public double getMinimumStepHeightForMaximumSwingHeight()
   {
      return 0.1;
   }

   /**
    * Parameters for setting swing time as a function of horizontal and vertical step displacement
    */
   
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

   @Override
   public double getMaximumStepHeightForMinimumSwingTime()
   {
      return 0.05;
   }

   @Override
   public double getMinimumStepHeightForMaximumSwingTime()
   {
      return 0.1;
   }

   /**
    * Parameter for calculating custom waypoint proportions
    */

   @Override public double getFootStubClearance()
   {
      return 0.06;
   }

   @Override public double getWaypointProportionShiftForStubAvoidance()
   {
      return 0.1;
   }
}
