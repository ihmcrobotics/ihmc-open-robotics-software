package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public AtlasSwingPlannerParameters()
   {
      this("");
   }

   public AtlasSwingPlannerParameters(String versionSuffix)
   {
      super(SwingPlannerParameterKeys.keys, AtlasSwingPlannerParameters.class, versionSuffix);
      loadUnsafe();
   }

   public static void main(String[] args)
   {
      AtlasSwingPlannerParameters parameters = new AtlasSwingPlannerParameters();
      parameters.save();
   }
}
