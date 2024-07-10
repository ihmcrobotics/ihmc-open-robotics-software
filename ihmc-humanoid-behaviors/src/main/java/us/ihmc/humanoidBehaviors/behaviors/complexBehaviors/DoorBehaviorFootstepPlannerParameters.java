package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class DoorBehaviorFootstepPlannerParameters extends StoredPropertySet implements DefaultFootstepPlannerParametersBasics
{
   public static final String versionSuffix = "_Door";

   public DoorBehaviorFootstepPlannerParameters()
   {
      this(versionSuffix);
   }

   public DoorBehaviorFootstepPlannerParameters(String versionSuffix)
   {
      super(DefaultFootstepPlannerParameters.keys, DoorBehaviorFootstepPlannerParameters.class, versionSuffix);
      load();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(DefaultFootstepPlannerParameters.keys, DoorBehaviorFootstepPlannerParameters.class);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}