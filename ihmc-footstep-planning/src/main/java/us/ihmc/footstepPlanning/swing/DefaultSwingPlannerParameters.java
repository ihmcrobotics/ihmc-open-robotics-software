package us.ihmc.footstepPlanning.swing;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public DefaultSwingPlannerParameters()
   {
      this(null);
   }

   private DefaultSwingPlannerParameters(SwingPlannerParametersReadOnly swingPlannerParameters)
   {
      super(SwingPlannerParameterKeys.keys, DefaultSwingPlannerParameters.class);

      if (swingPlannerParameters != null)
      {
         set(swingPlannerParameters);
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
      DefaultSwingPlannerParameters parameters = new DefaultSwingPlannerParameters();
      parameters.save();
   }}
