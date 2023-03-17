package us.ihmc.atlas.parameters;

import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasVisibilityGraphParameters extends StoredPropertySet implements VisibilityGraphsParametersBasics
{
   public AtlasVisibilityGraphParameters()
   {
      this("");
   }

   public AtlasVisibilityGraphParameters(String versionSuffix)
   {
      super(VisibilityGraphParametersKeys.keys, AtlasVisibilityGraphParameters.class, versionSuffix);
      loadUnsafe();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(VisibilityGraphParametersKeys.keys, AtlasVisibilityGraphParameters.class);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}
