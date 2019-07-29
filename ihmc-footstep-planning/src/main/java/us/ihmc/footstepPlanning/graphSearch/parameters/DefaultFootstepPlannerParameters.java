package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultFootstepPlannerParameters implements SaveableFootstepPlannerParameters
{
   private final StoredPropertySet propertySet;

   public DefaultFootstepPlannerParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   public DefaultFootstepPlannerParameters(String projectName, String pathToResources) // for robots and UIs that want their own defaults and saves
   {
      this(projectName, pathToResources, null);
   }

   public DefaultFootstepPlannerParameters(FootstepPlannerParametersReadOnly footstepPlannerParameters) // for message passing or temp access
   {
      this("ihmc-open-robotics-software", "ihmc-footstep-planning/src/main/resources", footstepPlannerParameters);
   }

   private DefaultFootstepPlannerParameters(String projectName,
                                            String pathToResources,
                                            FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      propertySet = new StoredPropertySet(FootstepPlannerParameterKeys.keys,
                                          getClass(),
                                          projectName,
                                          pathToResources);

      if (footstepPlannerParameters != null)
      {
         set(footstepPlannerParameters);
      }
      else
      {
         propertySet.load();
      }
   }

   @Override
   public StoredPropertySet getStoredPropertySet()
   {
      return propertySet;
   }
}
