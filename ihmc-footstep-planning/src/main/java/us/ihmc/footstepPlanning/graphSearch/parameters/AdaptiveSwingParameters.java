package us.ihmc.footstepPlanning.graphSearch.parameters;

/**
 * Parameters for setting swing times and trajectories from a footstep poses
 */
public interface AdaptiveSwingParameters
{
   /**
    * Parameters for setting swing waypoint height as a function of vertical step displacement
    */
   
   double getMinimumSwingHeight();
   
   double getMaximumSwingHeight();
   
   double getMaximumStepHeightForMinimumSwingHeight();
   
   double getMinimumStepHeightForMaximumSwingHeight();

   /**
    * Parameters for setting swing time as a function of step displacement
    */

   double getMinimumSwingTime();

   double getMaximumSwingTime();

   double getMaximumStepTranslationForMinimumSwingTime();

   double getMinimumStepTranslationForMaximumSwingTime();
   
   double getMaximumStepHeightForMinimumSwingTime();
   
   double getMinimumStepHeightForMaximumSwingTime();
}
