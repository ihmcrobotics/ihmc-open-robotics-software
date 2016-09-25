package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.testing.JUnitTools;

public class ExpandingPolytopeAlgorithmTest
{

   @Test
   public void testWithAPointInsideACube()
   {
      double epsilonRelative = 1e-5;
      ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
      //      PrintingExpandingPolytopeAlgorithmListener expandingPolytopeListener = new PrintingExpandingPolytopeAlgorithmListener();
      ExpandingPolytopeAlgorithmAssertListener expandingPolytopeListener = new ExpandingPolytopeAlgorithmAssertListener();

      expandingPolytopeAlgorithm.setExpandingPolytopeAlgorithmListener(expandingPolytopeListener);

      double halfLengthX = 0.5;
      double halfWidthY = 0.5;
      double halfHeightZ = 0.5;

      ConvexPolytope polytopeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
      ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructSinglePointPolytope(new Point3d());

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      transformOne.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(0.1, 0.2, 0.35);
      polytopeOne.applyTransform(transformOne);

      transformTwo.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformTwo.setTranslation(0.0, 0.0, 0.05);
      polytopeTwo.applyTransform(transformTwo);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      Point3d closestPointOnA = new Point3d();
      Point3d closestPointOnB = new Point3d();

      boolean areColliding = detector.arePolytopesColliding(polytopeOne, polytopeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);

      SimplexPolytope simplex = detector.getSimplex();
      int numberOfPointsOnSimplex = simplex.getNumberOfPoints();
      assertEquals(4, numberOfPointsOnSimplex);

      expandingPolytopeAlgorithm.setPolytopes(simplex, polytopeOne, polytopeTwo);
      Vector3d separatingDistanceVector = expandingPolytopeAlgorithm.computeExpandedPolytope(closestPointOnA, closestPointOnB);

      JUnitTools.assertTuple3dEquals(new Vector3d(0.0, 0.0, -0.2), separatingDistanceVector, 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, 0.0, -0.15), closestPointOnA, 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, 0.0, 0.05), closestPointOnB, 1e-7);
   }

   @Test
   public void testWithTwoCollidingCubes()
   {
      double epsilonRelative = 1e-5;
      ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
      ExpandingPolytopeAlgorithmAssertListener expandingPolytopeListener = new ExpandingPolytopeAlgorithmAssertListener();
      expandingPolytopeAlgorithm.setExpandingPolytopeAlgorithmListener(expandingPolytopeListener);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      translateObject(cubeOne, 1.4, 2.5, 3.8);
      translateObject(cubeTwo, 1.0, 2.0, 3.0);

      Point3d closestPointOnA = new Point3d();
      Point3d closestPointOnB = new Point3d();

      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);

      SimplexPolytope simplex = detector.getSimplex();
      int numberOfPointsOnSimplex = simplex.getNumberOfPoints();
      assertEquals(4, numberOfPointsOnSimplex);

      expandingPolytopeAlgorithm.setPolytopes(simplex, cubeOne, cubeTwo);
      Vector3d separatingDistanceVector = expandingPolytopeAlgorithm.computeExpandedPolytope(closestPointOnA, closestPointOnB);

      JUnitTools.assertTuple3dEquals(new Vector3d(0.0, 0.0, -0.2), separatingDistanceVector, 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(1.7, 2.75, 3.8), closestPointOnA, 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(1.7, 2.75, 4.0), closestPointOnB, 1e-7);
   }

   private void translateObject(ConvexPolytope polytope, double x, double y, double z)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(x, y, z);
      polytope.applyTransform(transform);
   }

   private void rotateObject(ConvexPolytope polytope, double rotX, double rotY, double rotZ)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEulerAndZeroTranslation(rotX, rotY, rotZ);
      polytope.applyTransform(transform);
   }

}
