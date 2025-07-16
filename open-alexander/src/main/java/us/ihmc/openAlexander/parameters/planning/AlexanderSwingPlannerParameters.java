package us.ihmc.openAlexander.parameters.planning;

import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AlexanderSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public AlexanderSwingPlannerParameters()
   {
      this("");
   }

   public AlexanderSwingPlannerParameters(String versionSuffix)
   {
      super(SwingPlannerParameterKeys.keys, AlexanderSwingPlannerParameters.class, versionSuffix);
      loadUnsafe();
   }

   public static void main(String[] args)
   {
      AlexanderSwingPlannerParameters parameters = new AlexanderSwingPlannerParameters();
      parameters.save();
   }
}
