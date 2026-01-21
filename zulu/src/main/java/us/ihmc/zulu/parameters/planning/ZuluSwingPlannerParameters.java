package us.ihmc.zulu.parameters.planning;

import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class ZuluSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public ZuluSwingPlannerParameters()
   {
      this("");
   }

   public ZuluSwingPlannerParameters(String versionSuffix)
   {
      super(SwingPlannerParameterKeys.keys, ZuluSwingPlannerParameters.class, versionSuffix);
      loadUnsafe();
   }

   public static void main(String[] args)
   {
      ZuluSwingPlannerParameters parameters = new ZuluSwingPlannerParameters();
      parameters.save();
   }
}
