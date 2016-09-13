package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;

public class SimpleCollisionDetector implements ScsCollisionDetector
{
   private final ArrayList<CollisionShape> collisionObjects = new ArrayList<CollisionShape>();

   // Temporary variables for computation:
   private final RigidBodyTransform transformOne = new RigidBodyTransform();
   private final RigidBodyTransform transformTwo = new RigidBodyTransform();
   private final Point3d centerOne = new Point3d();
   private final Point3d centerTwo = new Point3d();
   private final Vector3d centerToCenterVector = new Vector3d();

   private final Vector3d normalVector = new Vector3d();
   private final Vector3d tempVector = new Vector3d();

   @Override
   public void initialize()
   {
   }

   @Override
   public CollisionShapeFactory getShapeFactory()
   {
      return new SimpleCollisionShapeFactory(this);
   }

   @Override
   public void removeShape(Link link)
   {
   }

   @Override
   public CollisionShape lookupCollisionShape(Link link)
   {
      return null;
   }

   @Override
   public void performCollisionDetection(CollisionDetectionResult result)
   {
      int numberOfObjects = collisionObjects.size();

      for (int i = 0; i < numberOfObjects; i++)
      {
         CollisionShape objectOne = collisionObjects.get(i);

         for (int j = i + 1; j < numberOfObjects; j++)
         {
            CollisionShape objectTwo = collisionObjects.get(j);

            objectOne.getTransformToWorld(transformOne);
            objectTwo.getTransformToWorld(transformTwo);

            CollisionShapeDescription descriptionOne = objectOne.getDescription();
            CollisionShapeDescription descriptionTwo = objectTwo.getDescription();

            if ((descriptionOne instanceof SphereShapeDescription) && (descriptionTwo instanceof SphereShapeDescription))
            {
               doSphereSphereCollisionDetection(objectOne, (SphereShapeDescription) descriptionOne, objectTwo, (SphereShapeDescription) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof BoxShapeDescription) && (descriptionTwo instanceof BoxShapeDescription))
            {
               doBoxBoxCollisionDetection(objectOne, (BoxShapeDescription) descriptionOne, objectTwo, (BoxShapeDescription) descriptionTwo, result);
            }
         }
      }
   }

   private void doBoxBoxCollisionDetection(CollisionShape objectOne, BoxShapeDescription descriptionOne, CollisionShape objectTwo, BoxShapeDescription descriptionTwo, CollisionDetectionResult result)
   {
   }

   private void doSphereSphereCollisionDetection(CollisionShape objectOne, SphereShapeDescription descriptionOne, CollisionShape objectTwo, SphereShapeDescription descriptionTwo, CollisionDetectionResult result)
   {
      double radiusOne = descriptionOne.getRadius();
      double radiusTwo = descriptionTwo.getRadius();

      transformOne.getTranslation(centerOne);
      transformTwo.getTranslation(centerTwo);

      double distanceSquared = centerOne.distanceSquared(centerTwo);

      if (distanceSquared <= (radiusOne + radiusTwo) * (radiusOne + radiusTwo))
      {
         centerToCenterVector.sub(centerTwo, centerOne);

         Point3d pointOnOne = new Point3d(centerOne);
         normalVector.set(centerToCenterVector);
         normalVector.normalize();

         tempVector.set(normalVector);
         tempVector.scale(radiusOne);
         pointOnOne.add(tempVector);

         Point3d pointOnTwo = new Point3d(centerTwo);
         tempVector.set(normalVector);
         tempVector.scale(-radiusTwo);
         pointOnTwo.add(tempVector);

         double distance = Math.sqrt(distanceSquared) - radiusOne - radiusTwo;

         SimpleContactWrapper contacts = new SimpleContactWrapper(objectOne, objectTwo);
         contacts.addContact(pointOnOne, pointOnTwo, normalVector, distance);

         result.addContact(contacts);
      }
   }

   public void addShape(CollisionShape collisionShape)
   {
      collisionObjects.add(collisionShape);
   }

}
