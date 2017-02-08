package us.ihmc.simulationconstructionset.physics.collision.simple;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.Contacts;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.tools.testing.JUnitTools;

public class SimpleCollisionDetectorTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSphereToSphereCollisions()
   {
      SimpleCollisionDetector detector = new SimpleCollisionDetector();
      //      GdxCollisionDetector detector = new GdxCollisionDetector(10.0);

      CollisionShapeFactory shapeFactory = detector.getShapeFactory();

      double radiusOne = 0.5;
      CollisionShapeDescription<?> sphereOne = shapeFactory.createSphere(radiusOne);
      CollisionShapeDescription<?> sphereTwo = shapeFactory.createSphere(radiusOne);

      CollisionShape collideableObjectOne = shapeFactory.addShape(sphereOne);
      CollisionShape collideableObjectTwo = shapeFactory.addShape(sphereTwo);

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      double delta = 0.01;

      transformOne.setTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(1.0 + delta, 0.0, 0.0);

      collideableObjectOne.setTransformToWorld(transformOne);
      collideableObjectTwo.setTransformToWorld(transformTwo);

      CollisionDetectionResult result = new CollisionDetectionResult();
      detector.performCollisionDetection(result);

      assertEquals(0, result.getNumberOfCollisions());

      transformOne.setRotationEulerAndZeroTranslation(0.3, 0.7, 0.9);
      transformOne.setTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(1.0 - delta, 0.0, 0.0);

      collideableObjectOne.setTransformToWorld(transformOne);
      collideableObjectTwo.setTransformToWorld(transformTwo);

      result.clear();
      detector.performCollisionDetection(result);

      assertEquals(1, result.getNumberOfCollisions());
      Contacts collision = result.getCollision(0);

      assertEquals(1, collision.getNumberOfContacts());

      CollisionShape shapeA = collision.getShapeA();
      CollisionShape shapeB = collision.getShapeB();

      Point3d locationA = new Point3d();
      Point3d locationB = new Point3d();
      Vector3d normal = new Vector3d();

      double distance = collision.getDistance(0);
      collision.getWorldA(0, locationA);
      collision.getWorldB(0, locationB);
      collision.getWorldNormal(0, normal);
      if (!collision.isNormalOnA())
         normal.scale(-1.0);

      if (shapeA != collideableObjectOne)
      {
         CollisionShape tempShape = shapeA;
         shapeA = shapeB;
         shapeB = tempShape;

         Point3d tempLocation = locationA;
         locationA = locationB;
         locationB = tempLocation;

         normal.scale(-1.0);
      }

      assertTrue(shapeA == collideableObjectOne);
      assertTrue(shapeB == collideableObjectTwo);

      JUnitTools.assertTuple3dEquals(new Vector3d(-1.0, 0.0, 0.0), normal, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(0.49, 0.0, 0.0), locationA, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(0.5, 0.0, 0.0), locationB, 1e-7);
      assertEquals(-delta, distance, 1e-7);

      // Another sphere to sphere test
      transformOne.setTranslation(-0.7, -0.1, 0.13);
      transformOne.setTranslation(-0.4, 0.85, 0.3);

      collideableObjectOne.setTransformToWorld(transformOne);
      collideableObjectTwo.setTransformToWorld(transformTwo);

      result.clear();
      detector.performCollisionDetection(result);

      assertEquals(1, result.getNumberOfCollisions());
      collision = result.getCollision(0);

      assertEquals(1, collision.getNumberOfContacts());

      distance = collision.getDistance(0);
      collision.getWorldA(0, locationA);
      collision.getWorldB(0, locationB);
      collision.getWorldNormal(0, normal);
      if (!collision.isNormalOnA())
         normal.scale(-1.0);

      if (shapeA != collideableObjectOne)
      {
         CollisionShape tempShape = shapeA;
         shapeA = shapeB;
         shapeB = tempShape;

         Point3d tempLocation = locationA;
         locationA = locationB;
         locationB = tempLocation;

         normal.scale(-1.0);
      }

      assertTrue(shapeA == collideableObjectOne);
      assertTrue(shapeB == collideableObjectTwo);

      JUnitTools.assertTuple3dEquals(new Vector3d(0.40561610125071507, -0.8619342151577695, -0.3042120759380363), normal, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(-0.1971919493746425, 0.41903289242111524, 0.14789396203098185), locationA, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(-0.20280805062535753, 0.43096710757888473, 0.15210603796901814), locationB, 1e-7);
      assertEquals(-0.013845853834199007, distance, 1e-7);
   }

   //TODO: More and more vigorous tests...
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBoxToBoxCollisions()
   {
      SimpleCollisionDetector detector = new SimpleCollisionDetector();
      //      GdxCollisionDetector detector = new GdxCollisionDetector(10.0);

      CollisionShapeFactory shapeFactory = detector.getShapeFactory();

      double halfLengthX = 0.5;
      double halfWidthY = 0.6;
      double halfHeightZ = 0.7;
      CollisionShapeDescription<?> boxOne = shapeFactory.createBox(halfLengthX, halfWidthY, halfHeightZ);
      CollisionShapeDescription<?> boxTwo = shapeFactory.createBox(halfLengthX, halfWidthY, halfHeightZ);

      CollisionShape collideableObjectOne = shapeFactory.addShape(boxOne);
      CollisionShape collideableObjectTwo = shapeFactory.addShape(boxTwo);

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      double delta = 0.01;

      transformOne.setTranslation(0.0, 0.0, 0.0);
      transformTwo.setTranslation(1.0 + delta, 0.0, 0.0);

      collideableObjectOne.setTransformToWorld(transformOne);
      collideableObjectTwo.setTransformToWorld(transformTwo);

      CollisionDetectionResult result = new CollisionDetectionResult();
      detector.performCollisionDetection(result);

      assertEquals(0, result.getNumberOfCollisions());

      transformOne.setTranslation(0.0, 0.0, 0.0);
      transformTwo.setTranslation(1.0 - delta, 0.0, 0.0);

      collideableObjectOne.setTransformToWorld(transformOne);
      collideableObjectTwo.setTransformToWorld(transformTwo);

      result.clear();
      detector.performCollisionDetection(result);

      assertEquals(1, result.getNumberOfCollisions());
      Contacts collision = result.getCollision(0);

      assertEquals(1, collision.getNumberOfContacts());

      CollisionShape shapeA = collision.getShapeA();
      CollisionShape shapeB = collision.getShapeB();

      Point3d locationA = new Point3d();
      Point3d locationB = new Point3d();
      Vector3d normal = new Vector3d();

      double distance = collision.getDistance(0);
      collision.getWorldA(0, locationA);
      collision.getWorldB(0, locationB);
      collision.getWorldNormal(0, normal);
      if (!collision.isNormalOnA())
         normal.scale(-1.0);

      if (shapeA != collideableObjectOne)
      {
         CollisionShape tempShape = shapeA;
         shapeA = shapeB;
         shapeB = tempShape;

         Point3d tempLocation = locationA;
         locationA = locationB;
         locationB = tempLocation;

         normal.scale(-1.0);
      }

      assertTrue(shapeA == collideableObjectOne);
      assertTrue(shapeB == collideableObjectTwo);

      //      JUnitTools.assertTuple3dEquals(new Vector3d(-1.0, 0.0, 0.0), normal, 1e-7);
      //      JUnitTools.assertTuple3dEquals(new Vector3d(0.49, 0.0, 0.0), locationA, 1e-7);
      //      JUnitTools.assertTuple3dEquals(new Vector3d(0.5, 0.0, 0.0), locationB, 1e-7);
      assertEquals(-delta, distance, 1e-7);

      // Another sphere to sphere test
      transformOne.setTranslation(-0.7, -0.1, 0.13);
      transformTwo.setTranslation(-0.4, 0.85, 0.3);

      collideableObjectOne.setTransformToWorld(transformOne);
      collideableObjectTwo.setTransformToWorld(transformTwo);

      result.clear();
      detector.performCollisionDetection(result);

      assertEquals(1, result.getNumberOfCollisions());
      collision = result.getCollision(0);

      assertEquals(1, collision.getNumberOfContacts());

      distance = collision.getDistance(0);
      collision.getWorldA(0, locationA);
      collision.getWorldB(0, locationB);
      collision.getWorldNormal(0, normal);
      if (!collision.isNormalOnA())
         normal.scale(-1.0);

      if (shapeA != collideableObjectOne)
      {
         CollisionShape tempShape = shapeA;
         shapeA = shapeB;
         shapeB = tempShape;

         Point3d tempLocation = locationA;
         locationA = locationB;
         locationB = tempLocation;

         normal.scale(-1.0);
      }

      assertTrue(shapeA == collideableObjectOne);
      assertTrue(shapeB == collideableObjectTwo);

      //      JUnitTools.assertTuple3dEquals(new Vector3d(0.40561610125071507, -0.8619342151577695, -0.3042120759380363), normal, 1e-7);
      //      JUnitTools.assertTuple3dEquals(new Vector3d(-0.1971919493746425, 0.41903289242111524, 0.14789396203098185), locationA, 1e-7);
      //      JUnitTools.assertTuple3dEquals(new Vector3d(-0.20280805062535753, 0.43096710757888473, 0.15210603796901814), locationB, 1e-7);
      assertEquals(-0.25, distance, 1e-7);
   }

}
