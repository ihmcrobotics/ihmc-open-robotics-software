package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasFootstepPlannerParameters extends StoredPropertySet implements FootstepPlannerParametersBasics
{
   public AtlasFootstepPlannerParameters()
   {
      this("");
   }

   public AtlasFootstepPlannerParameters(String versionSuffix)
   {
      super(FootstepPlannerParameterKeys.keys, AtlasFootstepPlannerParameters.class, versionSuffix);
      loadUnsafe();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(FootstepPlannerParameterKeys.keys, AtlasFootstepPlannerParameters.class);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}