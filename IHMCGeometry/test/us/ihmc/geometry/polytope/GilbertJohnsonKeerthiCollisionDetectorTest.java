package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.random.RandomTools;

public class GilbertJohnsonKeerthiCollisionDetectorTest
{
   private static boolean VERBOSE = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeSupportPointOnMinkowskiDifference()
   {
      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(-1.0, -1.0, -1.0);
      cubeTwo.applyTransform(transform);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      Vector3D directionVector = new Vector3D(1.0, 1.0, 1.0);
      Point3D supportPoint = new Point3D();
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 2.0, 2.0), supportPoint, 1e-7);

      directionVector.set(-1.0, -1.0, -1.0);
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(0.0, 0.0, 0.0), supportPoint, 1e-7);

      directionVector.set(1.0, 1.0, -1.0);
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 2.0, 0.0), supportPoint, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsIntersecting()
   {
      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      translateObject(cubeOne, 1.0, 2.0, 3.0);
      translateObject(cubeTwo, 9.0, 6.0, 7.0);

      Point3D closestPointOnA = new Point3D();
      Point3D closestPointOnB = new Point3D();
      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertFalse(areColliding);

      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 3.0, 4.0), closestPointOnA, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(9.0, 6.0, 7.0), closestPointOnB, 1e-7);

      cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      translateObject(cubeOne, 1.4, 2.5, 3.8);
      translateObject(cubeTwo, 1.0, 2.0, 3.0);

      areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);
      //TODO: Test for closestPoints. Where should they be?
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectingOnARotatingCube()
   {
      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      Point3D closestPointOnA = new Point3D();
      Point3D closestPointOnB = new Point3D();

      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(100.0, 100.0, 0.5);
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(0.5, 1.0, 0.5);

      translateObject(cubeOne, 0.0, 0.0, -0.5); // zPlane is zero.
      translateObject(cubeTwo, 1.0, 2.0, 300.0);

      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertFalse(areColliding);

      assertEquals(0.0, closestPointOnA.getZ(), 1e-7);
      assertEquals(299.5, closestPointOnB.getZ(), 1e-7);

      assertTrue(closestPointOnA.getX() >= -100.0);
      assertTrue(closestPointOnA.getX() <= 100.0);
      assertTrue(closestPointOnA.getY() >= -100.0);
      assertTrue(closestPointOnA.getY() <= 100.0);

      assertEquals(299.5, closestPointOnA.distance(closestPointOnB), 1e-7);

      for (int i = 0; i < 100; i++)
      {
         rotateObject(cubeTwo, 0.0, 0.0, 0.01);
         areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
         assertFalse("" + cubeTwo, areColliding);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 30000)
   public void testExtensivelyPointToPolytope()
   {
      Random random = new Random(1776L);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      int numberOfPolytopesToTests = 500;

      // Generate a lot of random Polytopes. For each polytope generate a lot of points.
      // For each point do intersection and closest point test with GJK.
      //
      // If the point is outside, then march from the point to the point of intersection and make sure that each of those
      // points get projected to the same point.
      //
      // If the point is inside, then march from it to each vertex on the polytope and make sure that all of those points are inside
      // Continue marching outside of the polytope and make sure that that point projects to the vertex on the polytope that
      // was used to march outside.

      int numberInside = 0;
      int numberOutside = 0;

      for (int i = 0; i < numberOfPolytopesToTests; i++)
      {
         double xyzBoundary = RandomNumbers.nextDouble(random, 1000.0);
         double radius = RandomNumbers.nextDouble(random, 1.0, 20.0);
         int maxNumberOfPoints = 40;

         int numberOfPoints = random.nextInt(maxNumberOfPoints) + 1;
         ConvexPolytope polytope = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPoints, radius, xyzBoundary);

         int numberOfPointsToTests = 1000;

         for (int j = 0; j < numberOfPointsToTests; j++)
         {
            Point3D pointToProject = RandomTools.generateRandomPoint(random, xyzBoundary, xyzBoundary, xyzBoundary);
            ConvexPolytope randomPointPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointToProject);

            Point3D closestPointOnPolytope = new Point3D();
            Point3D closestPointOnPoint = new Point3D();
            boolean isInside = detector.arePolytopesColliding(polytope, randomPointPolytope, closestPointOnPolytope, closestPointOnPoint);

            EuclidCoreTestTools.assertTuple3DEquals(pointToProject, closestPointOnPoint, 1e-7);

            if (isInside)
            {
               EuclidCoreTestTools.assertTuple3DEquals(pointToProject, closestPointOnPolytope, 1e-4);
            }

            double distance = closestPointOnPolytope.distance(pointToProject);
            if (distance > 1e-4)
            {
               assertFalse(isInside);
            }

            // Make sure the projection projects to itself:
            ConvexPolytope closestPointCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(closestPointOnPolytope);
            Point3D projectItselfCheckProjection = new Point3D();
            boolean projectItselfCheck = detector.arePolytopesColliding(polytope, closestPointCheckPolytope, projectItselfCheckProjection, new Point3D());
            assertTrue(projectItselfCheck);
            double distanceBetweenTheTwoProjections = closestPointOnPolytope.distance(projectItselfCheckProjection);

            if (distanceBetweenTheTwoProjections > 1e-4)
            {
               printTroublesomeOne(polytope, pointToProject);
            }
            assertTrue("closestPointOnPolytope = " + closestPointOnPolytope + " distanceBetweenTheTwoProjections = " + distanceBetweenTheTwoProjections,
                  distanceBetweenTheTwoProjections < 1e-4);

            if (isInside)
            {
               numberInside++;
               //               System.out.println("Inside!");

               // For each vertex, go mostly to that vertex and make sure that point is still inside.
               // Then go beyond the vertex and make sure that point is outside of the polytope.

               int numberOfVertices = polytope.getNumberOfVertices();
               for (int k = 0; k < numberOfVertices; k++)
               {
                  PolytopeVertex vertex = polytope.getVertex(k);
                  Point3D vertexPosition = vertex.getPosition();

                  Vector3D vectorFromInteriorPointToVertex = new Vector3D();
                  vectorFromInteriorPointToVertex.sub(vertexPosition, pointToProject);

                  Point3D pointNearVertex = new Point3D(pointToProject);
                  Vector3D almostThere = new Vector3D(vectorFromInteriorPointToVertex);
                  almostThere.scale(0.99);
                  pointNearVertex.add(almostThere);

                  ConvexPolytope pointToCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointNearVertex);

                  Point3D pointShouldBeInPolytopeOne = new Point3D();
                  Point3D pointShouldBeInPolytopeTwo = new Point3D();
                  boolean shouldStillBeInside = detector.arePolytopesColliding(polytope, pointToCheckPolytope, pointShouldBeInPolytopeOne,
                        pointShouldBeInPolytopeTwo);
                  assertTrue(shouldStillBeInside);

                  EuclidCoreTestTools.assertTuple3DEquals(pointNearVertex, pointShouldBeInPolytopeOne, 1e-4);
                  EuclidCoreTestTools.assertTuple3DEquals(pointNearVertex, pointShouldBeInPolytopeTwo, 1e-4);

                  Point3D pointBeyondVertex = new Point3D(pointToProject);
                  Vector3D beyoneThere = new Vector3D(vectorFromInteriorPointToVertex);
                  beyoneThere.scale(1.01);
                  pointBeyondVertex.add(beyoneThere);

                  pointToCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointBeyondVertex);

                  Point3D pointShouldStayOutsidePolytope = new Point3D();
                  Point3D pointShouldProjectBackToPolytope = new Point3D();
                  boolean shouldBeOutside = detector.arePolytopesColliding(polytope, pointToCheckPolytope, pointShouldProjectBackToPolytope,
                        pointShouldStayOutsidePolytope);
                  assertFalse(shouldBeOutside);
                  assertTrue(Math.abs(pointBeyondVertex.distance(pointShouldStayOutsidePolytope)) < 1e-7);
               }
            }
            else
            {
               numberOutside++;
               //               System.out.println("Outside!");

               Vector3D vectorFromOutsidePointToProjection = new Vector3D();
               vectorFromOutsidePointToProjection.sub(closestPointOnPolytope, pointToProject);

               Point3D pointHalfwayToProjection = new Point3D(pointToProject);
               Vector3D halfWayToProjection = new Vector3D(vectorFromOutsidePointToProjection);
               halfWayToProjection.scale(0.5);
               pointHalfwayToProjection.add(halfWayToProjection);

               ConvexPolytope halfwayToProjectionPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointHalfwayToProjection);
               Point3D projectionFromHalfwayPoint = new Point3D();
               Point3D shouldBeSameHalfwayPoint = new Point3D();
               boolean shouldStillBeOutside = detector.arePolytopesColliding(polytope, halfwayToProjectionPolytope, projectionFromHalfwayPoint,
                     shouldBeSameHalfwayPoint);

               assertFalse(shouldStillBeOutside);
               EuclidCoreTestTools.assertTuple3DEquals(shouldBeSameHalfwayPoint, pointHalfwayToProjection, 1e-7);

               double originalProjectionDistance = pointToProject.distance(closestPointOnPolytope);
               double newProjectionDistance = pointToProject.distance(projectionFromHalfwayPoint);

               double newDistance = projectionFromHalfwayPoint.distance(closestPointOnPolytope);
               if ((newDistance > 1e-7) && (VERBOSE))
               {
                  System.err.println("\n-------------\nnumberOutside = " + numberOutside);
                  System.err.println("Polytope = " + polytope);

                  System.err.println("\nPoint to project = " + pointToProject);
                  System.err.println("Projection = " + closestPointOnPolytope);

                  System.err.println("\npointHalfwayToProjection = " + pointHalfwayToProjection);
                  System.err.println("Projection from halfway point = " + projectionFromHalfwayPoint);

                  System.err.println("\nOriginal distance = " + originalProjectionDistance);
                  System.err.println("New point distance = " + newProjectionDistance);
                  System.err.println("---------------------");
               }
               EuclidCoreTestTools.assertTuple3DEquals(projectionFromHalfwayPoint, closestPointOnPolytope, 1e-7);

            }
         }
      }

      //      System.out.println("numberInside = " + numberInside);
      //      System.out.println("numberOutside = " + numberOutside);
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 30000)
   public void testExtensivelyPolytopeToPolytope()
   {
      Random random = new Random(1776L);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      int numberOfPolytopesToTests = 500;

      // Generate a lot of random sets of two Polytopes.
      // For each set of two polytopes do intersection and closest point test with GJK.
      //
      // If the polytopes do not intersect, then march from the point to the point of intersection and make sure that each of those
      // points get projected to the same point.
      //
      // If the polytopes do intersect, then ....

      int numberColliding = 0;
      int numberNotColliding = 0;

      for (int i = 0; i < numberOfPolytopesToTests; i++)
      {
         double xyzBoundary = RandomNumbers.nextDouble(random, 500.0);
         double radius = RandomNumbers.nextDouble(random, 1.0, 10.0);
         int maxNumberOfPoints = 40;

         int numberOfPoints = random.nextInt(maxNumberOfPoints) + 1;
         ConvexPolytope polytopeOne = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPoints, radius, xyzBoundary);

         for (int j = 0; j < numberOfPolytopesToTests; j++)
         {
            ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPoints, radius, xyzBoundary);

            Point3D closestPointOnPolytopeOne = new Point3D();
            Point3D closestPointOnPolytopeTwo = new Point3D();
            boolean areColliding = detector.arePolytopesColliding(polytopeOne, polytopeTwo, closestPointOnPolytopeOne, closestPointOnPolytopeTwo);

            if (areColliding)
            {
               EuclidCoreTestTools.assertTuple3DEquals(closestPointOnPolytopeOne, closestPointOnPolytopeTwo, 1e-4);
            }

            double distance = closestPointOnPolytopeOne.distance(closestPointOnPolytopeTwo);
            if (distance > 1e-4)
            {
               assertFalse(areColliding);
            }

            // Make sure the projection projects to itself:
            checkPointProjectsToItself(detector, polytopeOne, closestPointOnPolytopeOne, 1e-4);
            checkPointProjectsToItself(detector, polytopeTwo, closestPointOnPolytopeTwo, 1e-4);

            if (areColliding)
            {
               numberColliding++;

               //               System.out.println("Are Colliding!");
               //TODO: Check these...
            }
            else
            {
               numberNotColliding++;

               Point3D pointHalfwayBetweenPolytopes = new Point3D(closestPointOnPolytopeOne);
               pointHalfwayBetweenPolytopes.add(closestPointOnPolytopeTwo);
               pointHalfwayBetweenPolytopes.scale(0.5);

               Point3D closestPointOnPolytopeOneCheck = checkPointProjectsToSamePoint(detector, polytopeOne, pointHalfwayBetweenPolytopes,
                     closestPointOnPolytopeOne, 1e-4);
               Point3D closestPointOnPolytopeTwoCheck = checkPointProjectsToSamePoint(detector, polytopeTwo, pointHalfwayBetweenPolytopes,
                     closestPointOnPolytopeTwo, 1e-4);

               double originalProjectionDistance = closestPointOnPolytopeOne.distance(closestPointOnPolytopeTwo);
               double newProjectionDistance = closestPointOnPolytopeOneCheck.distance(closestPointOnPolytopeTwoCheck);

               double newDistanceOne = closestPointOnPolytopeOneCheck.distance(closestPointOnPolytopeOne);
               if ((newDistanceOne > 1e-7) && (VERBOSE))
               {
                  System.err.println("PolytopeOne = " + polytopeOne);
                  System.err.println("PolytopeTwo = " + polytopeTwo);
                  System.err.println("Original distance = " + originalProjectionDistance);
                  System.err.println("New point distance = " + newProjectionDistance);
               }
               EuclidCoreTestTools.assertTuple3DEquals(closestPointOnPolytopeOneCheck, closestPointOnPolytopeOne, 1e-7);
               EuclidCoreTestTools.assertTuple3DEquals(closestPointOnPolytopeTwoCheck, closestPointOnPolytopeTwo, 1e-7);

            }
         }
      }

      if (VERBOSE)
      {
         System.out.println("number colliding = " + numberColliding);
         System.out.println("number not colliding = " + numberNotColliding);
      }

      // Make sure some are both colliding and not colliding
      assertTrue("numberColliding = " + numberColliding, numberColliding > 1000);
      assertTrue("numberNotColliding = " + numberNotColliding, numberNotColliding > 1000);
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test (timeout = 30000)
   public void testExtensivelyCylinderToPolytope()
   {
      Random random = new Random(1776L);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      int numberOfPolytopesToTests = 500;

      int numberColliding = 0;
      int numberNotColliding = 0;

      for (int i = 0; i < numberOfPolytopesToTests; i++)
      {
         double xyzBoundary = RandomNumbers.nextDouble(random, 20.0);
         double radius = RandomNumbers.nextDouble(random, 1.0, 5.0);
         double height = RandomNumbers.nextDouble(random, 1.0, 5.0);
         int maxNumberOfPoints = 20;

         int numberOfPoints = random.nextInt(maxNumberOfPoints) + 1;

         CylinderSupportingVertexHolder cylinder = new CylinderSupportingVertexHolder(radius, height);

         for (int j = 0; j < numberOfPolytopesToTests; j++)
         {
            ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPoints, radius, xyzBoundary);

            Point3D closestPointOnPolytopeOne = new Point3D();
            Point3D closestPointOnPolytopeTwo = new Point3D();
            boolean areColliding = detector.arePolytopesColliding(cylinder, polytopeTwo, closestPointOnPolytopeOne, closestPointOnPolytopeTwo);

            if (areColliding)
            {
               EuclidCoreTestTools.assertTuple3DEquals(closestPointOnPolytopeOne, closestPointOnPolytopeTwo, 1e-4);
            }

            double distance = closestPointOnPolytopeOne.distance(closestPointOnPolytopeTwo);
            if (distance > 1e-4)
            {
               assertFalse(areColliding);
            }

            // Make sure the projection projects to itself:
            checkPointProjectsToItself(detector, cylinder, closestPointOnPolytopeOne, 2e-4);
            checkPointProjectsToItself(detector, polytopeTwo, closestPointOnPolytopeTwo, 2e-4);

            if (areColliding)
            {
               numberColliding++;

               //               System.out.println("Are Colliding!");
               //TODO: Check these...
            }
            else
            {
               numberNotColliding++;

               Point3D pointHalfwayBetweenPolytopes = new Point3D(closestPointOnPolytopeOne);
               pointHalfwayBetweenPolytopes.add(closestPointOnPolytopeTwo);
               pointHalfwayBetweenPolytopes.scale(0.5);

               double epsilon = 0.005;
               Point3D closestPointOnPolytopeOneCheck = checkPointProjectsToSamePoint(detector, cylinder, pointHalfwayBetweenPolytopes,
                     closestPointOnPolytopeOne, epsilon);
               Point3D closestPointOnPolytopeTwoCheck = checkPointProjectsToSamePoint(detector, polytopeTwo, pointHalfwayBetweenPolytopes,
                     closestPointOnPolytopeTwo, epsilon);

               double originalProjectionDistance = closestPointOnPolytopeOne.distance(closestPointOnPolytopeTwo);
               double newProjectionDistance = closestPointOnPolytopeOneCheck.distance(closestPointOnPolytopeTwoCheck);

               double newDistanceOne = closestPointOnPolytopeOneCheck.distance(closestPointOnPolytopeOne);
               if ((newDistanceOne > 1e-7) && (VERBOSE))
               {
                  System.err.println("PolytopeOne = " + cylinder);
                  System.err.println("PolytopeTwo = " + polytopeTwo);
                  System.err.println("Original distance = " + originalProjectionDistance);
                  System.err.println("New point distance = " + newProjectionDistance);
               }
               EuclidCoreTestTools.assertTuple3DEquals(closestPointOnPolytopeOneCheck, closestPointOnPolytopeOne, epsilon);
               EuclidCoreTestTools.assertTuple3DEquals(closestPointOnPolytopeTwoCheck, closestPointOnPolytopeTwo, epsilon);

            }
         }
      }

      if (VERBOSE)
      {
         System.out.println("number colliding = " + numberColliding);
         System.out.println("number not colliding = " + numberNotColliding);
      }

      // Make sure some are both colliding and not colliding
      assertTrue("numberColliding = " + numberColliding, numberColliding > 1000);
      assertTrue("numberNotColliding = " + numberNotColliding, numberNotColliding > 1000);
   }

   private Point3D checkPointProjectsToSamePoint(GilbertJohnsonKeerthiCollisionDetector detector, SupportingVertexHolder polytope, Point3D pointToProject,
         Point3D pointItShouldProjectTo, double epsilon)
   {
      ConvexPolytope pointToProjectPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointToProject);
      Point3D projectionPoint = new Point3D();
      Point3D shouldBeSamePointAsPointToProject = new Point3D();
      boolean shouldStillBeOutside = detector.arePolytopesColliding(polytope, pointToProjectPolytope, projectionPoint, shouldBeSamePointAsPointToProject);

      if (pointToProject.distance(pointItShouldProjectTo) > 1e-4)
      {
         assertFalse(shouldStillBeOutside);
      }

      EuclidCoreTestTools.assertTuple3DEquals(pointItShouldProjectTo, projectionPoint, epsilon);

      return projectionPoint;
   }

   private void checkPointProjectsToItself(GilbertJohnsonKeerthiCollisionDetector detector, SupportingVertexHolder polytope, Point3D closestPointOnPolytope,
         double epsilon)
   {
      ConvexPolytope closestPointCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(closestPointOnPolytope);
      Point3D projectItselfCheckProjection = new Point3D();
      boolean projectItselfCheck = detector.arePolytopesColliding(polytope, closestPointCheckPolytope, projectItselfCheckProjection, new Point3D());
      //      assertTrue(projectItselfCheck);
      double distanceBetweenTheTwoProjections = closestPointOnPolytope.distance(projectItselfCheckProjection);

      if (distanceBetweenTheTwoProjections > epsilon)
      {
         printTroublesomeOne(polytope, closestPointOnPolytope);
      }
      assertTrue(" distanceBetweenTheTwoProjections = " + distanceBetweenTheTwoProjections, distanceBetweenTheTwoProjections < epsilon);
   }

   private void printTroublesomeOne(SupportingVertexHolder polytope, Point3D pointToProject)
   {
      if (VERBOSE)
      {
         System.err.println("\n\nTroublesome point to polytope projection!!");
         System.err.println("Polytope = " + polytope);
         System.err.println("Point to project = " + pointToProject);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublesomePointToLineSegments()
   {
      Point3D lineSegmentPointA = new Point3D(-125.59926862231721, -680.1428915432899, 298.6543939303482);
      Point3D lineSegmentPointB = new Point3D(-125.95786649149319, -680.9859637318334, 296.6726500977285);
      Point3D pointToProject = new Point3D(-129.21231825319057, -656.2911659793187, 258.51753051888545);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3D(196.36355237922527, 44.83905852516866, -518.7281201463553);
      lineSegmentPointB = new Point3D(194.6218923133981, 44.08041418195433, -521.1005287800097);
      pointToProject = new Point3D(-891.6670220834754, 782.4709354629376, -617.9936823374843);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3D(196.36355237922527, 44.83905852516866, -518.7281201463553);
      lineSegmentPointB = new Point3D(194.6218923133981, 44.08041418195433, -521.1005287800097);
      pointToProject = new Point3D(437.12828528833893, -211.7311999187434, -614.5155752244341);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3D(196.36355237922527, 44.83905852516866, -518.7281201463553);
      lineSegmentPointB = new Point3D(194.6218923133981, 44.08041418195433, -521.1005287800097);
      pointToProject = new Point3D(801.8465242152599, -868.722674580723, -675.152141794304);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3D(-498.51606150874665, 757.2404015991474, -96.97900033315614);
      lineSegmentPointB = new Point3D(-498.8640447105517, 755.4124172795532, -94.21489685836693);
      pointToProject = new Point3D(-559.9574965281266, -544.0982107907117, -965.280473529281);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3D(-69.92326753629632, -803.7425861766396, -724.1784756661798);
      lineSegmentPointB = new Point3D(-70.04267037420692, -802.8267104204476, -723.4258524791725);
      pointToProject = new Point3D(772.880695200153, -583.6111408356501, -858.3481670598765);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublesomePointToTriangle()
   {
      Point3D trianglePointA = new Point3D(747.3053277443838, 870.081212940944, -887.5577972908313);
      Point3D trianglePointB = new Point3D(744.680386956109, 871.7731054517318, -888.5741060630728);
      Point3D trianglePointC = new Point3D(747.1694588869868, 869.9135409393845, -887.601628701288);
      Point3D pointToProject = new Point3D(550.3577591322485, 829.9756877678701, -461.9923107729438);

      doPointToTriangleProjectionTest(trianglePointA, trianglePointB, trianglePointC, pointToProject, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublesomePointToTetragon()
   {
      Point3D tetragonPointA = new Point3D(-498.42171267030795, 940.7427864661465, 476.39846668568447);
      Point3D tetragonPointB = new Point3D(-495.85886916155226, 940.6676228557476, 477.11939206103483);
      Point3D tetragonPointC = new Point3D(-497.22720354039853, 939.3655423334133, 476.5484075466428);
      Point3D tetragonPointD = new Point3D(-496.02959529366643, 939.9368132428073, 478.51963102396314);
      Point3D pointToProject = new Point3D(756.3802841804468, -14.952705301326773, -346.6583066638593);

      doPointToTetragonProjectionTest(tetragonPointA, tetragonPointB, tetragonPointC, tetragonPointD, pointToProject, false);
   }

   private void doPointToEdgeProjectionTest(Point3D lineSegmentPointA, Point3D lineSegmentPointB, Point3D pointToProject, boolean verbose)
   {
      doPointToSimplePolytopeTest(pointToProject, verbose, lineSegmentPointA, lineSegmentPointB);
   }

   private void doPointToTriangleProjectionTest(Point3D trianglePointA, Point3D trianglePointB, Point3D trianglePointC, Point3D pointToProject, boolean verbose)
   {
      doPointToSimplePolytopeTest(pointToProject, verbose, trianglePointA, trianglePointB, trianglePointC);
   }

   private void doPointToTetragonProjectionTest(Point3D tetragonPointA, Point3D tetragonPointB, Point3D tetragonPointC, Point3D tetragonPointD,
         Point3D pointToProject, boolean verbose)
   {
      doPointToSimplePolytopeTest(pointToProject, verbose, tetragonPointA, tetragonPointB, tetragonPointC, tetragonPointD);
   }

   private void doPointToSimplePolytopeTest(Point3D pointToProject, boolean verbose, Point3D... polytopePoints)
   {
      if (verbose)
      {
         System.out.println("\nPoint to simple polytope projection test.");
         for (int i = 0; i < polytopePoints.length; i++)
         {
            System.out.println("polytopePoint" + i + " = " + polytopePoints[i]);

         }
         System.out.println("pointToProject = " + pointToProject);
      }
      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();

      if (verbose)
      {
         detector.setGilbertJohnsonKeerthiCollisionDetectorListener(new PrintingGilbertJohnsonKeerthiCollisionDetectorListener());
      }
      else
      {
         GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
         detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);
      }

      ConvexPolytope polytope = new ConvexPolytope();

      polytope.addVertices(polytopePoints);

      ConvexPolytope singlePointPolygon = ConvexPolytopeConstructor.constructSinglePointPolytope(pointToProject);

      Point3D closestPointOnPolytope = new Point3D();
      Point3D pointOnAToPack = new Point3D();

      boolean areColliding = detector.arePolytopesColliding(singlePointPolygon, polytope, pointOnAToPack, closestPointOnPolytope);
      assertFalse(areColliding);

      // Make sure the projection projects to itself:
      ConvexPolytope closestPointCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(closestPointOnPolytope);
      Point3D projectItselfCheckProjection = new Point3D();
      boolean projectItselfCheck = detector.arePolytopesColliding(polytope, closestPointCheckPolytope, projectItselfCheckProjection, new Point3D());
      assertTrue(projectItselfCheck);
      EuclidCoreTestTools.assertTuple3DEquals(closestPointOnPolytope, projectItselfCheckProjection, 1e-7);

      Vector3D vectorFromOutsidePointToProjection = new Vector3D();
      vectorFromOutsidePointToProjection.sub(closestPointOnPolytope, pointToProject);

      Point3D pointHalfwayToProjection = new Point3D(pointToProject);
      Vector3D halfWayToProjection = new Vector3D(vectorFromOutsidePointToProjection);
      halfWayToProjection.scale(0.5);
      pointHalfwayToProjection.add(halfWayToProjection);

      ConvexPolytope polytopeHalfwayToProjection = ConvexPolytopeConstructor.constructSinglePointPolytope(pointHalfwayToProjection);
      Point3D projectionFromHalfwayPoint = new Point3D();

      areColliding = detector.arePolytopesColliding(polytopeHalfwayToProjection, polytope, pointOnAToPack, projectionFromHalfwayPoint);
      assertFalse(areColliding);

      double originalProjectionDistance = closestPointOnPolytope.distance(pointToProject);
      double newProjectionDistance = projectionFromHalfwayPoint.distance(pointToProject);

      // Double check by just computing the projection directly here.

      Point3D checkProjectedPoint = bruteForceProjectOntoPolytope(pointToProject, polytopePoints);
      Point3D checkProjectionFromHalfwayPoint = bruteForceProjectOntoPolytope(pointHalfwayToProjection, polytopePoints);

      double distanceBetweenProjections = projectionFromHalfwayPoint.distance(closestPointOnPolytope);
      if (distanceBetweenProjections > 1e-7)
      {

         System.err.println("Polytope = " + polytope);
         System.err.println("Point to project = " + pointToProject);
         System.err.println("pointHalfwayToProjection = " + pointHalfwayToProjection);
         System.err.println("Projection = " + closestPointOnPolytope);
         System.err.println("Projection from halfway point = " + projectionFromHalfwayPoint);
         System.err.println("Original distance = " + originalProjectionDistance);
         System.err.println("New point distance = " + newProjectionDistance);

         System.err.println("\ncheckProjectedPoint = " + checkProjectedPoint);
         System.err.println("Check projection from halfway point = " + checkProjectionFromHalfwayPoint);
      }

      EuclidCoreTestTools.assertTuple3DEquals(projectionFromHalfwayPoint, closestPointOnPolytope, 1e-7);

   }

   private Point3D bruteForceProjectOntoPolytope(Point3D pointToProject, Point3D[] polytopePoints)
   {
      if (polytopePoints.length == 1)
      {
         return new Point3D(polytopePoints[0]);
      }
      else if (polytopePoints.length == 2)
      {
         return bruteForceProjectOntoLineSegment(pointToProject, polytopePoints[0], polytopePoints[1]);
      }
      else if (polytopePoints.length == 3)
      {
         return bruteForceProjectOntoTriangle(pointToProject, polytopePoints[0], polytopePoints[1], polytopePoints[2]);
      }
      else if (polytopePoints.length == 4)
      {
         return bruteForceProjectOntoTetragon(pointToProject, polytopePoints[0], polytopePoints[1], polytopePoints[2], polytopePoints[3]);
      }
      throw new RuntimeException("Only support up to 4 vertices");
   }

   private Point3D bruteForceProjectOntoTetragon(Point3D pointToProject, Point3D pointA, Point3D pointB, Point3D pointC, Point3D pointD)
   {
      Point3D bestProjection = new Point3D();
      double bestDistance = Double.POSITIVE_INFINITY;

      Point3D triangleProjection = bruteForceProjectOntoTriangle(pointToProject, pointA, pointB, pointC);
      double distance = pointToProject.distance(triangleProjection);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(triangleProjection);
      }

      triangleProjection = bruteForceProjectOntoTriangle(pointToProject, pointA, pointB, pointD);
      distance = pointToProject.distance(triangleProjection);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(triangleProjection);
      }

      triangleProjection = bruteForceProjectOntoTriangle(pointToProject, pointA, pointC, pointD);
      distance = pointToProject.distance(triangleProjection);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(triangleProjection);
      }

      triangleProjection = bruteForceProjectOntoTriangle(pointToProject, pointB, pointC, pointD);
      distance = pointToProject.distance(triangleProjection);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(triangleProjection);
      }

      //TODO: See if inside Tetragon alread and if so, return the point.

      return bestProjection;
   }

   private Point3D bruteForceProjectOntoTriangle(Point3D pointToProject, Point3D trianglePointA, Point3D trianglePointB, Point3D trianglePointC)
   {
      Point3D bestProjection = new Point3D();
      double bestDistance = Double.POSITIVE_INFINITY;

      Point3D projectedPoint = bruteForceProjectOntoLineSegment(pointToProject, trianglePointA, trianglePointB);
      double distance = pointToProject.distance(projectedPoint);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(projectedPoint);
      }

      projectedPoint = bruteForceProjectOntoLineSegment(pointToProject, trianglePointA, trianglePointC);
      distance = pointToProject.distance(projectedPoint);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(projectedPoint);
      }

      projectedPoint = bruteForceProjectOntoLineSegment(pointToProject, trianglePointB, trianglePointC);
      distance = pointToProject.distance(projectedPoint);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(projectedPoint);
      }

      //TODO: Project onto the plane ABC and make sure inside

      Vector3D normal = new Vector3D();
      Vector3D abVector = new Vector3D();
      abVector.sub(trianglePointB, trianglePointA);
      Vector3D acVector = new Vector3D();
      abVector.sub(trianglePointC, trianglePointA);

      normal.cross(abVector, acVector);

      Vector3D aPVector = new Vector3D();
      aPVector.sub(pointToProject, trianglePointA);

      Vector3D tempCross = new Vector3D();
      tempCross.cross(abVector, aPVector);

      Vector3D tempCrossTwo = new Vector3D();
      tempCross.cross(aPVector, acVector);

      double gamma = tempCross.dot(normal) / (normal.dot(normal));
      double beta = tempCrossTwo.dot(normal) / (normal.dot(normal));
      double alpha = 1.0 - beta - gamma;

      if ((alpha >= 0.0) && (alpha <= 1.0) && (beta > 0.0) && (beta <= 1.0) && (gamma >= 0.0) && (gamma <= 1.0))
      {
         projectedPoint = new Point3D();
         Point3D tempPoint = new Point3D();

         tempPoint.set(trianglePointA);
         tempPoint.scale(alpha);
         projectedPoint.add(tempPoint);

         tempPoint.set(trianglePointB);
         tempPoint.scale(beta);
         projectedPoint.add(tempPoint);

         tempPoint.set(trianglePointC);
         tempPoint.scale(gamma);
         projectedPoint.add(tempPoint);

         bestProjection = projectedPoint;
      }

      return bestProjection;
   }

   public Point3D bruteForceProjectOntoLineSegment(Point3D pointToProject, Point3D pointA, Point3D pointB)
   {
      LineSegment3d lineSegment = new LineSegment3d(pointA, pointB);
      Point3D projectedPoint = new Point3D();
      lineSegment.orthogonalProjection(pointToProject, projectedPoint);

      return projectedPoint;
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(GilbertJohnsonKeerthiCollisionDetectorTest.class);
   }
}
