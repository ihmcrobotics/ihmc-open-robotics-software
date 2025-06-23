package us.ihmc.openAlexander.parameters.planning;

import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AlexanderVisibilityGraphParameters extends StoredPropertySet implements VisibilityGraphsParametersBasics
{
   public AlexanderVisibilityGraphParameters()
   {
      this("");
   }

   public AlexanderVisibilityGraphParameters(String versionSuffix)
   {
      super(VisibilityGraphParametersKeys.keys, AlexanderVisibilityGraphParameters.class, versionSuffix);
      loadUnsafe();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(VisibilityGraphParametersKeys.keys, AlexanderVisibilityGraphParameters.class);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}
