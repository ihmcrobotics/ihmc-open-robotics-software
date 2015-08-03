package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.LinkedHashMap;

import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class RigidBodyToIndexMap
{
   private final RigidBody[] rigidBodies;
   private final String[] rigidBodyNames;

   private final LinkedHashMap<String, Integer> rigidBodyIndexByNameMap = new LinkedHashMap<String, Integer>();
   private final LinkedHashMap<String, RigidBody> rigidBodyByNameMap = new LinkedHashMap<String, RigidBody>();
   

   public RigidBodyToIndexMap(RigidBody rootBody)
   {
      rigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      rigidBodyNames = new String[rigidBodies.length];
      for (int index = 0; index < rigidBodies.length; index++)
      {
         RigidBody rigidBody = rigidBodies[index];
         rigidBodyNames[index] = rigidBody.getName();

         rigidBodyIndexByNameMap.put(rigidBody.getName(), index);
         rigidBodyByNameMap.put(rigidBody.getName(), rigidBody);

      }
   }

   public int getIndexByName(String rigidBodyName)
   {
      return rigidBodyIndexByNameMap.get(rigidBodyName);
   }
   
   public String getNameByIndex(int index)
   {
      return rigidBodyNames[index];
   }
   
   public RigidBody getRigidBodyByName(String name)
   {
      return rigidBodyByNameMap.get(name);
   }

}
