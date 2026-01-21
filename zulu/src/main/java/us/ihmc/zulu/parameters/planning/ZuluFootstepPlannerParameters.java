package us.ihmc.zulu.parameters.planning;

import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class ZuluFootstepPlannerParameters extends StoredPropertySet implements DefaultFootstepPlannerParametersBasics
{
   public ZuluFootstepPlannerParameters()
   {
      this("");
   }

   public ZuluFootstepPlannerParameters(String versionSuffix)
   {
      super(DefaultFootstepPlannerParameters.keys, ZuluFootstepPlannerParameters.class, versionSuffix);
      loadUnsafe();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(DefaultFootstepPlannerParameters.keys, ZuluFootstepPlannerParameters.class);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}
