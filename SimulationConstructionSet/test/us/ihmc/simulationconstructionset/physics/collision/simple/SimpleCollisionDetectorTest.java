package us.ihmc.simulationconstructionset.physics.collision.simple;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.DetectedCollision;

public class SimpleCollisionDetectorTest
{

   @Test
   public void testSphereCollisions()
   {
      SimpleCollisionDetector detector = new SimpleCollisionDetector();

      CollisionShapeFactory shapeFactory = detector.getShapeFactory();

      double radiusOne = 0.5;
      CollisionShapeDescription sphereOne = shapeFactory.createSphere(radiusOne);
      CollisionShapeDescription sphereTwo = shapeFactory.createSphere(radiusOne);

      CollisionObject collideableObjectOne = new CollisionObject(sphereOne);
      CollisionObject collideableObjectTwo = new CollisionObject(sphereTwo);

      detector.addShape(collideableObjectOne);
      detector.addShape(collideableObjectTwo);

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      double delta = 0.001;

      transformOne.setTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(1.0 + delta, 0.0, 0.0);

      collideableObjectOne.setTransformToWorld(transformOne);
      collideableObjectTwo.setTransformToWorld(transformTwo);

      CollisionDetectionResult result = new CollisionDetectionResult();
      detector.performCollisionDetection(result);

      assertEquals(0, result.getNumberOfCollisions());

      transformOne.setTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(1.0 - delta, 0.0, 0.0);

      collideableObjectOne.setTransformToWorld(transformOne);
      collideableObjectTwo.setTransformToWorld(transformTwo);

      result.clear();
      detector.performCollisionDetection(result);

      assertEquals(1, result.getNumberOfCollisions());
      DetectedCollision collision = result.getCollision(0);
      CollisionShape shapeA = collision.getShapeA();
      CollisionShape shapeB = collision.getShapeB();



   }

}
