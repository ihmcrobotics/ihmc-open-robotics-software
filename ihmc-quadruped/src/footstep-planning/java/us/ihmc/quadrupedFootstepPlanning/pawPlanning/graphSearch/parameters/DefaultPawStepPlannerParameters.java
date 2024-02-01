package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultPawStepPlannerParameters extends StoredPropertySet implements PawStepPlannerParametersBasics
{
   public DefaultPawStepPlannerParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   private DefaultPawStepPlannerParameters(PawStepPlannerParametersReadOnly pawPlannerParameters)
   {
      super(PawStepPlannerParameterKeys.keys, DefaultPawStepPlannerParameters.class);

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
