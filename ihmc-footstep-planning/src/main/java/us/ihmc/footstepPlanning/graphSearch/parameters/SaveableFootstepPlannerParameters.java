package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public interface SaveableFootstepPlannerParameters extends FootstepPlannerParametersBasics
{
   @Override
   StoredPropertySet getStoredPropertySet();
}