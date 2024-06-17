package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.StoredPropertySetDelegate;

/**
 * Allows for easily swapping different parameter sets while not changing the API of existing code.
 */
public class FootstepPlannerParametersDelegate extends StoredPropertySetDelegate implements DefaultFootstepPlannerParametersBasics
{
   public void setParametersToDelegate(DefaultFootstepPlannerParametersBasics footstepPlannerParameters)
   {
      setStoredPropertySetToDelegate(footstepPlannerParameters);
   }
}
