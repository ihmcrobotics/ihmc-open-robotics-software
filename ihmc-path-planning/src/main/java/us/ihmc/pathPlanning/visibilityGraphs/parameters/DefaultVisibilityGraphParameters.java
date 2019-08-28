package us.ihmc.pathPlanning.visibilityGraphs.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultVisibilityGraphParameters extends StoredPropertySet implements VisibilityGraphsParametersBasics
{
   public DefaultVisibilityGraphParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   public DefaultVisibilityGraphParameters(String projectName, String pathToResources) // for robots and UIs that want their own defaults and saves
   {
      this(projectName, pathToResources, null);
   }

   public DefaultVisibilityGraphParameters(VisibilityGraphsParametersReadOnly parameters) // for message passing or temp access
   {
      this("ihmc-open-robotics-software", "ihmc-footstep-planning/src/main/resources", parameters);
   }

   private DefaultVisibilityGraphParameters(String projectName, String pathToResources, VisibilityGraphsParametersReadOnly parameters)
   {
      super(VisibilityGraphParametersKeys.keys, DefaultVisibilityGraphParameters.class, projectName, pathToResources);

      if (parameters != null)
      {
         set(parameters);
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
      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();
      parameters.save();
   }
}
