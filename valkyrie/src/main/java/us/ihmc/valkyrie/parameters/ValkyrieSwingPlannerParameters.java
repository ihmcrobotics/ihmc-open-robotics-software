package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class ValkyrieSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public ValkyrieSwingPlannerParameters()
   {
      this("");
   }

   public ValkyrieSwingPlannerParameters(String versionSuffix)
   {
      super(SwingPlannerParameterKeys.keys, ValkyrieSwingPlannerParameters.class, versionSuffix);
      loadUnsafe();
   }

   public static void main(String[] args)
   {
      ValkyrieSwingPlannerParameters parameters = new ValkyrieSwingPlannerParameters();
      parameters.save();
   }
}
