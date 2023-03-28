package us.ihmc.pathPlanning.visibilityGraphs.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultVisibilityGraphParameters extends StoredPropertySet implements VisibilityGraphsParametersBasics
{
   public DefaultVisibilityGraphParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   private DefaultVisibilityGraphParameters(VisibilityGraphsParametersReadOnly parameters)
   {
      super(VisibilityGraphParametersKeys.keys, DefaultVisibilityGraphParameters.class);

      if (parameters != null)
      {
         set(parameters);
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
      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();
      parameters.save();
   }
}
