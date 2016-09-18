package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
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
      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();

      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      translateObject(cubeOne, 1.0, 2.0, 3.0);
      translateObject(cubeTwo, 9.0, 6.0, 7.0);

      Point3d closestPointOnA = new Point3d();
      Point3d closestPointOnB = new Point3d();
      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertFalse(areColliding);

      JUnitTools.assertPoint3dEquals("", new Point3d(2.0, 3.0, 4.0), closestPointOnA, 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(9.0, 6.0, 7.0), closestPointOnB, 1e-7);

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

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectingOnALotOfCubes()
   {
      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      Point3d closestPointOnA = new Point3d();
      Point3d closestPointOnB = new Point3d();

      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(100.0, 100.0, 0.5);
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(0.5, 1.0, 0.5);

      translateObject(cubeOne, 0.0, 0.0, -0.5); // zPlane is zero.
      translateObject(cubeTwo, 1.0, 2.0, 300.0);

      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertFalse(areColliding);

      JUnitTools.assertPoint3dEquals("", new Point3d(0.99502487, 1.980198, 0.0), closestPointOnA, 1e-4);
      JUnitTools.assertPoint3dEquals("", new Point3d(0.99502487, 1.980198, 299.5), closestPointOnB, 1e-7);

      assertEquals(299.5, closestPointOnA.distance(closestPointOnB), 1e-7);

      for (int i = 0; i < 100; i++)
      {
         rotateObject(cubeTwo, 0.0, 0.0, 0.01);
         //         System.out.println("Test " + i);

         //         if (i == 11)
         //         {
         //            System.out.println("ThisOne ");
         //         }
         areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
         assertFalse("" + cubeTwo, areColliding);
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testExtensivelyPointToPolytope()
   {
      Random random = new Random(1776L);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();

      int numberOfPolytopesToTests = 1000;

      // Generate a lot of random Polytopes. For each polytope generate a lot of points.
      // For each point do intersection and closest point test with GJK. 
      // 
      // If the point is outside, then march from the point to the point of intersection and make sure that each of those
      // points get projected to the same point.
      //
      // If the point is inside, then march from it to each vertex on the polytope and make sure that all of those points are inside
      // Continue marching outside of the polytope and make sure that that point projects to the vertex on the polytope that
      // was used to march outside.

      for (int i = 0; i < numberOfPolytopesToTests; i++)
      {
         double xyzBoundary = 1000.0;
         double radius = 1.7;
         int numberOfPoints = 4;
         ConvexPolytope polytope = constructRandomSphereOutlinedPolytope(random, numberOfPoints, radius, xyzBoundary);

         int numberOfPointsToTests = 1000;
         int numberInside = 0;
         int numberOutside = 0;
         for (int j = 0; j < numberOfPointsToTests; j++)
         {
            Point3d pointToProject = RandomTools.generateRandomPoint(random, xyzBoundary, xyzBoundary, xyzBoundary);
            ConvexPolytope randomPointPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointToProject);

            Point3d closestPointOnPolytope = new Point3d();
            Point3d closestPointOnPoint = new Point3d();
            boolean isInside = detector.arePolytopesColliding(polytope, randomPointPolytope, closestPointOnPolytope, closestPointOnPoint);

            JUnitTools.assertTuple3dEquals(pointToProject, closestPointOnPoint, 1e-7);

            boolean isInsideCheck = Math.abs(pointToProject.distance(closestPointOnPolytope)) < 1e-7;
            assertEquals(isInside, isInsideCheck);

            // Make sure the projection projects to itself:
            ConvexPolytope closestPointCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(closestPointOnPolytope);
            Point3d projectItselfCheckProjection = new Point3d();
            boolean projectItselfCheck = detector.arePolytopesColliding(polytope, closestPointCheckPolytope, projectItselfCheckProjection, new Point3d());
            assertTrue(projectItselfCheck);
            double distanceBetweenTheTwoProjections = closestPointOnPolytope.distance(projectItselfCheckProjection);

            if (distanceBetweenTheTwoProjections > 1e-7)
            {
               printTroublesomeOne(polytope, pointToProject);
            }
            assertTrue(distanceBetweenTheTwoProjections < 1e-7);

            if (isInside)
            {
               System.out.println("Inside!");

               numberInside++;
               // For each vertex, go mostly to that vertex and make sure that point is still inside.
               // Then go beyond the vertex and make sure that point projects back to the vertex.

               int numberOfVertices = polytope.getNumberOfVertices();
               for (int k = 0; k < numberOfVertices; k++)
               {
                  PolytopeVertex vertex = polytope.getVertex(k);
                  Point3d vertexPosition = vertex.getPosition();

                  Vector3d vectorFromInteriorPointToVertex = new Vector3d();
                  vectorFromInteriorPointToVertex.sub(vertexPosition, pointToProject);

                  Point3d pointNearVertex = new Point3d(pointToProject);
                  Vector3d almostThere = new Vector3d(vectorFromInteriorPointToVertex);
                  almostThere.scale(0.99);
                  pointNearVertex.add(almostThere);

                  ConvexPolytope pointToCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointNearVertex);

                  Point3d pointShouldBeInPolytopeOne = new Point3d();
                  Point3d pointShouldBeInPolytopeTwo = new Point3d();
                  boolean shouldStillBeInside = detector.arePolytopesColliding(polytope, pointToCheckPolytope, pointShouldBeInPolytopeOne,
                        pointShouldBeInPolytopeTwo);
                  assertTrue(shouldStillBeInside);

                  JUnitTools.assertTuple3dEquals(pointNearVertex, pointShouldBeInPolytopeOne, 1e-7);
                  JUnitTools.assertTuple3dEquals(pointNearVertex, pointShouldBeInPolytopeTwo, 1e-7);

                  Point3d pointBeyondVertex = new Point3d(pointToProject);
                  Vector3d beyoneThere = new Vector3d(vectorFromInteriorPointToVertex);
                  beyoneThere.scale(1.01);
                  pointBeyondVertex.add(beyoneThere);

                  pointToCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointBeyondVertex);

                  Point3d pointShouldStayOutsidePolytope = new Point3d();
                  Point3d pointShouldProjectToVertex = new Point3d();
                  boolean shouldBeOutside = detector.arePolytopesColliding(polytope, pointToCheckPolytope, pointShouldProjectToVertex,
                        pointShouldStayOutsidePolytope);
                  assertFalse(shouldBeOutside);
                  assertTrue(Math.abs(pointBeyondVertex.distance(pointShouldStayOutsidePolytope)) < 1e-7);
                  assertTrue(Math.abs(vertexPosition.distance(pointShouldProjectToVertex)) < 1e-7);
               }
            }
            else
            {
               numberOutside++;
               //               System.out.println("Outside!");
               Vector3d vectorFromOutsidePointToProjection = new Vector3d();
               vectorFromOutsidePointToProjection.sub(closestPointOnPolytope, pointToProject);

               Point3d pointHalfwayToProjection = new Point3d(pointToProject);
               Vector3d halfWayToProjection = new Vector3d(vectorFromOutsidePointToProjection);
               halfWayToProjection.scale(0.5);
               pointHalfwayToProjection.add(halfWayToProjection);

               ConvexPolytope halfwayToProjectionPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(pointHalfwayToProjection);
               Point3d projectionFromHalfwayPoint = new Point3d();
               Point3d shouldBeSameHalfwayPoint = new Point3d();
               boolean shouldStillBeOutside = detector.arePolytopesColliding(polytope, halfwayToProjectionPolytope, projectionFromHalfwayPoint,
                     shouldBeSameHalfwayPoint);

               assertFalse(shouldStillBeOutside);
               JUnitTools.assertTuple3dEquals(shouldBeSameHalfwayPoint, pointHalfwayToProjection, 1e-7);

               double originalProjectionDistance = pointToProject.distance(closestPointOnPolytope);
               double newProjectionDistance = pointToProject.distance(projectionFromHalfwayPoint);

               double newDistance = projectionFromHalfwayPoint.distance(closestPointOnPolytope);
               if (newDistance > 1e-7)
               {
                  System.err.println("numberOutside = " + numberOutside);
                  System.err.println("Polytope = " + polytope);
                  System.err.println("Point to project = " + pointToProject);
                  System.err.println("pointHalfwayToProjection = " + pointHalfwayToProjection);
                  System.err.println("Projection = " + closestPointOnPolytope);
                  System.err.println("Projection from halfway point = " + projectionFromHalfwayPoint);
                  System.err.println("Original distance = " + originalProjectionDistance);
                  System.err.println("New point distance = " + newProjectionDistance);
               }
               JUnitTools.assertTuple3dEquals(projectionFromHalfwayPoint, closestPointOnPolytope, 1e-7);

            }
         }
      }
   }

   private void printTroublesomeOne(ConvexPolytope polytope, Point3d pointToProject)
   {
      System.err.println("\n\nTroublesome point to polytope projection!!");
      System.err.println("Polytope = " + polytope);
      System.err.println("Point to project = " + pointToProject);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublesomePointToLineSegments()
   {
      Point3d lineSegmentPointA = new Point3d(-125.59926862231721, -680.1428915432899, 298.6543939303482);
      Point3d lineSegmentPointB = new Point3d(-125.95786649149319, -680.9859637318334, 296.6726500977285);
      Point3d pointToProject = new Point3d(-129.21231825319057, -656.2911659793187, 258.51753051888545);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3d(196.36355237922527, 44.83905852516866, -518.7281201463553);
      lineSegmentPointB = new Point3d(194.6218923133981, 44.08041418195433, -521.1005287800097);
      pointToProject = new Point3d(-891.6670220834754, 782.4709354629376, -617.9936823374843);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3d(196.36355237922527, 44.83905852516866, -518.7281201463553);
      lineSegmentPointB = new Point3d(194.6218923133981, 44.08041418195433, -521.1005287800097);
      pointToProject = new Point3d(437.12828528833893, -211.7311999187434, -614.5155752244341);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3d(196.36355237922527, 44.83905852516866, -518.7281201463553);
      lineSegmentPointB = new Point3d(194.6218923133981, 44.08041418195433, -521.1005287800097);
      pointToProject = new Point3d(801.8465242152599, -868.722674580723, -675.152141794304);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3d(-498.51606150874665, 757.2404015991474, -96.97900033315614);
      lineSegmentPointB = new Point3d(-498.8640447105517, 755.4124172795532, -94.21489685836693);
      pointToProject = new Point3d(-559.9574965281266, -544.0982107907117, -965.280473529281);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

      lineSegmentPointA = new Point3d(-69.92326753629632, -803.7425861766396, -724.1784756661798);
      lineSegmentPointB = new Point3d(-70.04267037420692, -802.8267104204476, -723.4258524791725);
      pointToProject = new Point3d(772.880695200153, -583.6111408356501, -858.3481670598765);

      doPointToEdgeProjectionTest(lineSegmentPointA, lineSegmentPointB, pointToProject, false);

   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublesomePointToTriangle()
   {
      Point3d trianglePointA = new Point3d(747.3053277443838, 870.081212940944, -887.5577972908313);
      Point3d trianglePointB = new Point3d(744.680386956109, 871.7731054517318, -888.5741060630728);
      Point3d trianglePointC = new Point3d(747.1694588869868, 869.9135409393845, -887.601628701288);
      Point3d pointToProject = new Point3d(550.3577591322485, 829.9756877678701, -461.9923107729438);

      doPointToTriangleProjectionTest(trianglePointA, trianglePointB, trianglePointC, pointToProject, false);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublesomePointToTetragon()
   {
      Point3d tetragonPointA = new Point3d(-498.42171267030795, 940.7427864661465, 476.39846668568447);
      Point3d tetragonPointB = new Point3d(-495.85886916155226, 940.6676228557476, 477.11939206103483);
      Point3d tetragonPointC = new Point3d(-497.22720354039853, 939.3655423334133, 476.5484075466428);
      Point3d tetragonPointD = new Point3d(-496.02959529366643, 939.9368132428073, 478.51963102396314);
      Point3d pointToProject = new Point3d(756.3802841804468, -14.952705301326773, -346.6583066638593);

      doPointToTetragonProjectionTest(tetragonPointA, tetragonPointB, tetragonPointC, tetragonPointD, pointToProject, true);
   }

   private void doPointToEdgeProjectionTest(Point3d lineSegmentPointA, Point3d lineSegmentPointB, Point3d pointToProject, boolean verbose)
   {
      doPointToSimplePolytopeTest(pointToProject, verbose, lineSegmentPointA, lineSegmentPointB);
   }

   private void doPointToTriangleProjectionTest(Point3d trianglePointA, Point3d trianglePointB, Point3d trianglePointC, Point3d pointToProject, boolean verbose)
   {
      doPointToSimplePolytopeTest(pointToProject, verbose, trianglePointA, trianglePointB, trianglePointC);
   }

   private void doPointToTetragonProjectionTest(Point3d tetragonPointA, Point3d tetragonPointB, Point3d tetragonPointC, Point3d tetragonPointD,
         Point3d pointToProject, boolean verbose)
   {
      doPointToSimplePolytopeTest(pointToProject, verbose, tetragonPointA, tetragonPointB, tetragonPointC);
   }

   private void doPointToSimplePolytopeTest(Point3d pointToProject, boolean verbose, Point3d... polytopePoints)
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
         detector.setGilbertJohnsonKeerthiCollisionDetectorListener(new PrintingGilbertJohnsonKeerthiCollisionDetectorListener());

      ConvexPolytope polytope = new ConvexPolytope();

      polytope.addVertices(polytopePoints);

      ConvexPolytope singlePointPolygon = ConvexPolytopeConstructor.constructSinglePointPolytope(pointToProject);

      Point3d closestPointOnPolytope = new Point3d();
      Point3d pointOnAToPack = new Point3d();

      boolean areColliding = detector.arePolytopesColliding(singlePointPolygon, polytope, pointOnAToPack, closestPointOnPolytope);
      assertFalse(areColliding);

      // Make sure the projection projects to itself:
      ConvexPolytope closestPointCheckPolytope = ConvexPolytopeConstructor.constructSinglePointPolytope(closestPointOnPolytope);
      Point3d projectItselfCheckProjection = new Point3d();
      boolean projectItselfCheck = detector.arePolytopesColliding(polytope, closestPointCheckPolytope, projectItselfCheckProjection, new Point3d());
      assertTrue(projectItselfCheck);
      //      JUnitTools.assertTuple3dEquals(closestPointOnPolytope, projectItselfCheckProjection, 1e-7);

      Vector3d vectorFromOutsidePointToProjection = new Vector3d();
      vectorFromOutsidePointToProjection.sub(closestPointOnPolytope, pointToProject);

      Point3d pointHalfwayToProjection = new Point3d(pointToProject);
      Vector3d halfWayToProjection = new Vector3d(vectorFromOutsidePointToProjection);
      halfWayToProjection.scale(0.5);
      pointHalfwayToProjection.add(halfWayToProjection);

      ConvexPolytope polytopeHalfwayToProjection = ConvexPolytopeConstructor.constructSinglePointPolytope(pointHalfwayToProjection);
      Point3d projectionFromHalfwayPoint = new Point3d();

      areColliding = detector.arePolytopesColliding(polytopeHalfwayToProjection, polytope, pointOnAToPack, projectionFromHalfwayPoint);
      assertFalse(areColliding);

      double originalProjectionDistance = closestPointOnPolytope.distance(pointToProject);
      double newProjectionDistance = projectionFromHalfwayPoint.distance(pointToProject);

      // Double check by just computing the projection directly here. 

      Point3d checkProjectedPoint = bruteForceProjectOntoPolytope(pointToProject, polytopePoints);
      Point3d checkProjectionFromHalfwayPoint = bruteForceProjectOntoPolytope(pointHalfwayToProjection, polytopePoints);

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

      JUnitTools.assertTuple3dEquals(projectionFromHalfwayPoint, closestPointOnPolytope, 1e-7);

   }

   private Point3d bruteForceProjectOntoPolytope(Point3d pointToProject, Point3d[] polytopePoints)
   {
      if (polytopePoints.length == 1)
      {
         return new Point3d(polytopePoints[0]);
      }
      else if (polytopePoints.length == 2)
      {
         LineSegment3d lineSegment = new LineSegment3d(polytopePoints[0], polytopePoints[1]);
         Point3d projectedPoint = new Point3d();
         lineSegment.getProjectionOntoLineSegment(pointToProject, projectedPoint);

         return projectedPoint;
      }
      else if (polytopePoints.length == 3)
      {
         return bruteForceProjectOntoTriangle(pointToProject, polytopePoints[0], polytopePoints[1], polytopePoints[2]);
      }
      else if (polytopePoints.length == 4)
      {
         throw new RuntimeException("Implement me!");
      }
      throw new RuntimeException("Only support up to 4 vertices");
   }

   private Point3d bruteForceProjectOntoTriangle(Point3d pointToProject, Point3d trianglePointA, Point3d trianglePointB, Point3d trianglePointC)
   {
      Point3d bestProjection = new Point3d();
      double bestDistance = Double.POSITIVE_INFINITY;

      double distance = pointToProject.distance(trianglePointA);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(trianglePointA);
      }

      distance = pointToProject.distance(trianglePointB);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(trianglePointB);
      }

      distance = pointToProject.distance(trianglePointC);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(trianglePointC);
      }

      Point3d projectedPoint = new Point3d();
      LineSegment3d lineSegment = new LineSegment3d(trianglePointA, trianglePointB);
      lineSegment.getProjectionOntoLineSegment(pointToProject, projectedPoint);

      distance = pointToProject.distance(projectedPoint);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(projectedPoint);
      }

      projectedPoint = new Point3d();
      lineSegment = new LineSegment3d(trianglePointA, trianglePointC);
      lineSegment.getProjectionOntoLineSegment(pointToProject, projectedPoint);

      distance = pointToProject.distance(projectedPoint);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(projectedPoint);
      }

      projectedPoint = new Point3d();
      lineSegment = new LineSegment3d(trianglePointB, trianglePointC);
      lineSegment.getProjectionOntoLineSegment(pointToProject, projectedPoint);

      distance = pointToProject.distance(projectedPoint);
      if (distance < bestDistance)
      {
         bestDistance = distance;
         bestProjection.set(projectedPoint);
      }

      //TODO: Project onto the plane ABC and make sure inside

      return bestProjection;
   }

   private static ConvexPolytope constructRandomSphereOutlinedPolytope(Random random, int numberOfPoints, double radius, double xyzBoundary)
   {
      ConvexPolytope polytope = new ConvexPolytope();

      Point3d sphereCenter = RandomTools.generateRandomPoint(random, xyzBoundary, xyzBoundary, xyzBoundary);

      for (int i = 0; i < numberOfPoints; i++)
      {
         Vector3d randomVector = RandomTools.generateRandomVector(random, radius);
         Point3d point = new Point3d(sphereCenter);
         point.add(randomVector);

         //TODO: Need to connect the edges later once they are used in the algorithm!!
         polytope.addVertex(point);
      }

      return polytope;
   }

}
