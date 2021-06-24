package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class DoorBehaviorFootstepPlannerParameters extends StoredPropertySet implements FootstepPlannerParametersBasics
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String PATH_TO_RESOURCES = "atlas/src/main/resources";
   public static final String fileNameSuffix = "_Door";


   public DoorBehaviorFootstepPlannerParameters()
   {
      this(PROJECT_NAME, PATH_TO_RESOURCES, fileNameSuffix);
   }

   public DoorBehaviorFootstepPlannerParameters(String projectName, String pathToResources, String fileNameSuffix)
   {
      super(FootstepPlannerParameterKeys.keys, DoorBehaviorFootstepPlannerParameters.class, projectName, pathToResources, fileNameSuffix);
      load();

      System.out.println();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      new DoorBehaviorFootstepPlannerParameters();

//      StoredPropertySet storedPropertySet = new StoredPropertySet(FootstepPlannerParameterKeys.keys,
//                                                                  DoorBehaviorFootstepPlannerParameters.class,
//                                                                  PROJECT_NAME,
//                                                                  PATH_TO_RESOURCES);
//      storedPropertySet.loadUnsafe();
//      storedPropertySet.save();
   }
}