package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultPawStepPlannerParameters extends StoredPropertySet implements PawStepPlannerParametersBasics
{
   public DefaultPawStepPlannerParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   public DefaultPawStepPlannerParameters(String projectName, String pathToResources) // for robots and UIs that want their own defaults and saves
   {
      this(projectName, pathToResources, null);
   }

   public DefaultPawStepPlannerParameters(PawStepPlannerParametersReadOnly pawPlannerParameters) // for message passing or temp access
   {
      this("ihmc-open-robotics-software", "ihmc-quadruped-footstep-planning/src/main/resources", pawPlannerParameters);
   }

   private DefaultPawStepPlannerParameters(String projectName, String pathToResources, PawStepPlannerParametersReadOnly pawPlannerParameters)
   {
      super(PawStepPlannerParameterKeys.keys, DefaultPawStepPlannerParameters.class, projectName, pathToResources);

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
      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      parameters.save();
   }
}
