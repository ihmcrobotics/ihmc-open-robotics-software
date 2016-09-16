package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class GilbertJohnsonKeerthiCollisionDetectorTest
{

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeSupportPointOnMinkowskiDifference()
   {
      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(-1.0, -1.0, -1.0);
      cubeTwo.applyTransform(transform);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();

      Vector3d directionVector = new Vector3d(1.0, 1.0, 1.0);
      Point3d supportPoint = new Point3d();
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      JUnitTools.assertPoint3dEquals("", new Point3d(2.0, 2.0, 2.0), supportPoint, 1e-7);

      directionVector.set(-1.0, -1.0, -1.0);
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      JUnitTools.assertPoint3dEquals("", new Point3d(0.0, 0.0, 0.0), supportPoint, 1e-7);

      directionVector.set(1.0, 1.0, -1.0);
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      JUnitTools.assertPoint3dEquals("", new Point3d(2.0, 2.0, 0.0), supportPoint, 1e-7);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsIntersecting()
   {
      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      transformOne.setTranslation(1.0, 2.0, 3.0);
      transformTwo.setTranslation(9.0, 6.0, 7.0);

      cubeOne.applyTransform(transformOne);
      cubeTwo.applyTransform(transformTwo);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();

      Point3d closestPointOnA = new Point3d();
      Point3d closestPointOnB = new Point3d();
      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertFalse(areColliding);

      JUnitTools.assertPoint3dEquals("", new Point3d(2.0, 3.0, 4.0), closestPointOnA, 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(9.0, 6.0, 7.0), closestPointOnB, 1e-7);

      cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      transformOne.setTranslation(1.4, 2.5, 3.8);
      transformTwo.setTranslation(1.0, 2.0, 3.0);

      cubeOne.applyTransform(transformOne);
      cubeTwo.applyTransform(transformTwo);

      areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);
      //TODO: Test for closestPoints. Where should they be?

   }
}
