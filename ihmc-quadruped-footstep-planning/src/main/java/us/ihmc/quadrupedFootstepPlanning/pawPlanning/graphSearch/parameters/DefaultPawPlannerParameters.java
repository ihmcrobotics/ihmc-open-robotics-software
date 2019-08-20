package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultPawPlannerParameters extends StoredPropertySet implements PawPlannerParametersBasics
{
   public DefaultPawPlannerParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   public DefaultPawPlannerParameters(String projectName, String pathToResources) // for robots and UIs that want their own defaults and saves
   {
      this(projectName, pathToResources, null);
   }

   public DefaultPawPlannerParameters(PawPlannerParametersReadOnly pawPlannerParameters) // for message passing or temp access
   {
      this("ihmc-open-robotics-software", "ihmc-quadruped-footstep-planning/src/main/resources", pawPlannerParameters);
   }

   private DefaultPawPlannerParameters(String projectName, String pathToResources, PawPlannerParametersReadOnly pawPlannerParameters)
   {
      super(PawPlannerParameterKeys.keys, DefaultPawPlannerParameters.class, projectName, pathToResources);

      if (pawPlannerParameters != null)
      {
         set(pawPlannerParameters);
      }
      else
      {
         load();
      }
   }

   /**
    * Run to update file with new parameters.
    */
   public static void main(String[] args)
   {
      DefaultPawPlannerParameters parameters = new DefaultPawPlannerParameters();
      parameters.save();
   }
}
