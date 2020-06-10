package us.ihmc.footstepPlanning.swing;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public DefaultSwingPlannerParameters()
   {
      this(null);
   }

   public DefaultSwingPlannerParameters(String projectName, String pathToResources) // for robots and UIs that want their own defaults and saves
   {
      this(projectName, pathToResources, null);
   }

   public DefaultSwingPlannerParameters(SwingPlannerParametersReadOnly swingPlannerParametersReadOnly) // for message passing or temp access
   {
      this("ihmc-open-robotics-software", "ihmc-footstep-planning/src/main/resources", swingPlannerParametersReadOnly);
   }

   private DefaultSwingPlannerParameters(String projectName, String pathToResources, SwingPlannerParametersReadOnly swingPlannerParameters)
   {
      super(SwingPlannerParameterKeys.keys, DefaultSwingPlannerParameters.class, projectName, pathToResources);

      if (swingPlannerParameters != null)
      {
         set(swingPlannerParameters);
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
      DefaultSwingPlannerParameters parameters = new DefaultSwingPlannerParameters();
      parameters.save();
   }}
