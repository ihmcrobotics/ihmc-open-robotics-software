package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultFootstepPlannerParameters extends StoredPropertySet implements FootstepPlannerParametersBasics
{
   public DefaultFootstepPlannerParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   private DefaultFootstepPlannerParameters(FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      super(FootstepPlannerParameterKeys.keys, DefaultFootstepPlannerParameters.class);

      if (footstepPlannerParameters != null)
      {
         set(footstepPlannerParameters);
      }
      else
      {
         loadUnsafe();
      }
   }

   /**
    * Run to update file with new parameters.
    */
   public static void main(String[] args)
   {
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      parameters.save();
   }
}
