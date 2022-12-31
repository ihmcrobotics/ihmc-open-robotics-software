package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface AStarBodyPathPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getRollCostWeight()
   {
      return get(rollCostWeight);
   }

   default double getInclineCostWeight()
   {
      return get(inclineCostWeight);
   }

   default double getInclineCostDeadband()
   {
      return get(inclineCostDeadband);
   }

   default double getMaxIncline()
   {
      return get(maxIncline);
   }

   default boolean getCheckForCollisions()
   {
      return get(checkForCollisions);
   }

   default boolean getComputeSurfaceNormalCost()
   {
      return get(computeSurfaceNormalCost);
   }

   default boolean getPerformSmoothing()
   {
      return get(performSmoothing);
   }
}
