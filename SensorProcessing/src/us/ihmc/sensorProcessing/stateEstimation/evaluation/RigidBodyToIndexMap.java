package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;

public class RigidBodyToIndexMap
{
   private final RigidBody[] rigidBodies;
   
   private final Map<RigidBody, Integer> rigidBodyToIndexMap = new LinkedHashMap<RigidBody, Integer>();

   public RigidBodyToIndexMap(RigidBody rootBody)
   {
      rigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      
      for (int index=0; index<rigidBodies.length; index++)
      {
         RigidBody rigidBody = rigidBodies[index];   
         rigidBodyToIndexMap.put(rigidBody, index);
      }
   }

   public RigidBody lookupRigidBody(int index)
   {
      return rigidBodies[index];
   }
   
   public int lookupIndexOfRigidBody(RigidBody rigidBody)
   {
      return rigidBodyToIndexMap.get(rigidBody);
   }
   
}
