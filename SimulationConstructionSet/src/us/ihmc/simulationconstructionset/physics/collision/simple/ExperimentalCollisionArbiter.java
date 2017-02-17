package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.Collection;
import java.util.LinkedHashMap;

import us.ihmc.simulationconstructionset.physics.CollisionArbiter;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.Contacts;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;

public class ExperimentalCollisionArbiter implements CollisionArbiter
{   
   private final LinkedHashMap<CollisionShape, LinkedHashMap<CollisionShape, SimpleContactWrapper>> contactMap = new LinkedHashMap<>();
   
   @Override
   public void processNewCollisions(CollisionDetectionResult newCollisions)
   {
      int numberOfCollisions = newCollisions.getNumberOfCollisions();
      
      for (int i=0; i<numberOfCollisions; i++)
      {
         Contacts collision = newCollisions.getCollision(i);
         
         CollisionShape shapeA = collision.getShapeA();
         CollisionShape shapeB = collision.getShapeB();
         
         SimpleContactWrapper shapeABContacts = getOrCreateContacts(shapeA, shapeB);

//         shapeABContacts.addAllReplaceNearby(collision);
//         shapeABContacts.addAll(collision);
         shapeABContacts.set(collision);
      }
   }

   private SimpleContactWrapper getOrCreateContacts(CollisionShape shapeA, CollisionShape shapeB)
   {
      LinkedHashMap<CollisionShape, SimpleContactWrapper> shapeAContacts = contactMap.get(shapeA);
      LinkedHashMap<CollisionShape, SimpleContactWrapper> shapeBContacts = contactMap.get(shapeB);
      
      if ((shapeAContacts == null) && (shapeBContacts == null))
      {
         shapeAContacts = new LinkedHashMap<>();
         SimpleContactWrapper contactsBetweenShapesAAndB = new SimpleContactWrapper(shapeA, shapeB);
         shapeAContacts.put(shapeB, contactsBetweenShapesAAndB);
         contactMap.put(shapeA, shapeAContacts);
         return contactsBetweenShapesAAndB;
      }
      
      if (shapeAContacts == null)
      {
         LinkedHashMap<CollisionShape, SimpleContactWrapper> temp = shapeAContacts;
         shapeAContacts = shapeBContacts;
         shapeBContacts = shapeAContacts;
         
         CollisionShape tempShape = shapeA;
         shapeA = shapeB;
         shapeB = shapeA;
      }
      
      SimpleContactWrapper contactsBetweenShapesAAndB = shapeAContacts.get(shapeB);
      if (contactsBetweenShapesAAndB == null)
      {
         contactsBetweenShapesAAndB = new SimpleContactWrapper(shapeA, shapeB);
         shapeAContacts.put(shapeB, contactsBetweenShapesAAndB);
      }

      return contactsBetweenShapesAAndB;
   }

   @Override
   public CollisionDetectionResult getCollisions()
   {
      CollisionDetectionResult results = new CollisionDetectionResult();
      
      Collection<LinkedHashMap<CollisionShape, SimpleContactWrapper>> valuesOne = contactMap.values();
      
      for (LinkedHashMap<CollisionShape, SimpleContactWrapper> valueOne : valuesOne)
      {
         Collection<SimpleContactWrapper> valuesTwo = valueOne.values();
         for (SimpleContactWrapper contacts : valuesTwo)
         {
            results.addContact(contacts);
         }
      }

      return results;
   }

}
