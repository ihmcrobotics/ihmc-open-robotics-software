package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
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
      Vector3d separatingDistanceVector = new Vector3d();
      expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

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

      translateObject(cubeOne, new Vector3d(1.4, 2.5, 3.8));
      translateObject(cubeTwo, new Vector3d(1.0, 2.0, 3.0));

      Point3d closestPointOnA = new Point3d();
      Point3d closestPointOnB = new Point3d();

      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);

      SimplexPolytope simplex = detector.getSimplex();
      int numberOfPointsOnSimplex = simplex.getNumberOfPoints();
      assertEquals(4, numberOfPointsOnSimplex);

      expandingPolytopeAlgorithm.setPolytopes(simplex, cubeOne, cubeTwo);
      Vector3d separatingDistanceVector = new Vector3d();
      expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

      JUnitTools.assertTuple3dEquals(new Vector3d(0.0, 0.0, -0.2), separatingDistanceVector, 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(1.7, 2.75, 3.8), closestPointOnA, 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(1.7, 2.75, 4.0), closestPointOnB, 1e-7);
   }

   @Test
   public void testExtensivelyWithTwoCubes()
   {
      Random random = new Random(1999L);

      double epsilonRelative = 1e-5;
      ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
      ExpandingPolytopeAlgorithmAssertListener expandingPolytopeListener = new ExpandingPolytopeAlgorithmAssertListener();
      expandingPolytopeAlgorithm.setExpandingPolytopeAlgorithmListener(expandingPolytopeListener);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      int numberOfTests = 1000;

      int numberOfCollisions = 0;
      for (int i = 0; i < numberOfTests; i++)
      {
         ConvexPolytope cubeOne = generateRandomCube(random);
         ConvexPolytope cubeTwo = generateRandomCube(random);

         rotateObject(cubeOne, RandomTools.generateRandomDouble(random, Math.PI), RandomTools.generateRandomDouble(random, Math.PI),
               RandomTools.generateRandomDouble(random, Math.PI));
         rotateObject(cubeTwo, RandomTools.generateRandomDouble(random, Math.PI), RandomTools.generateRandomDouble(random, Math.PI),
               RandomTools.generateRandomDouble(random, Math.PI));

         translateObject(cubeOne, RandomTools.generateRandomVector(random, new Vector3d(5.0, -7.0, 10.0), new Vector3d(7.0, -5.0, 12.0)));
         translateObject(cubeTwo, RandomTools.generateRandomVector(random, new Vector3d(5.0, -7.0, 10.0), new Vector3d(7.0, -5.0, 12.0)));

         Point3d closestPointOnA = new Point3d();
         Point3d closestPointOnB = new Point3d();

         boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);

         if (areColliding)
         {
            //            numberOfCollisions++;
            SimplexPolytope simplex = detector.getSimplex();
            int numberOfPointsOnSimplex = simplex.getNumberOfPoints();

            //            assertEquals(4, numberOfPointsOnSimplex);
            if (numberOfPointsOnSimplex != 4)
            {

            }

            else
            {
               numberOfCollisions++;
               expandingPolytopeAlgorithm.setPolytopes(simplex, cubeOne, cubeTwo);
               Vector3d separatingDistanceVector = new Vector3d();
               expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

               assertEquals(separatingDistanceVector.length(), closestPointOnA.distance(closestPointOnB), 1e-7);
            }
         }
      }

      assertTrue("numberOfCollisions = " + numberOfCollisions, numberOfCollisions > 500);
   }

   @Test
   public void testTroublesomeCubes()
   {
      double epsilonRelative = 1e-5;
      ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
      ExpandingPolytopeAlgorithmAssertListener expandingPolytopeListener = new ExpandingPolytopeAlgorithmAssertListener();
      expandingPolytopeAlgorithm.setExpandingPolytopeAlgorithmListener(expandingPolytopeListener);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      double[][] cubeOneVertices = new double[][] {{-0.5903738864022472, 0.04082374023374287, 0.10539748532999185},
            {-0.5778817564514865, -0.04473636929849633, 0.12059633456225372}, {-0.6790574742661268, -0.06840353680005194, 0.07052246269743494},
            {-0.6915496042168874, 0.017156572732187247, 0.055323613465173074}, {-0.5642605772611933, 0.03569432285950272, 0.05505917669100756},
            {-0.5517684473104327, -0.04986578667273647, 0.07025802592326942}, {-0.6529441651250729, -0.07353295417429209, 0.020184154058450643},
            {-0.6654362950758336, 0.0120271553579471, 0.00498530482618878}};

      double[][] cubeTwoVertices = new double[][] {{-100.005, -100.0, -0.005}, {99.995, -100.0, -0.005}, {99.995, 100.0, -0.005}, {-100.005, 100.0, -0.005},
            {-100.005, -100.0, 0.005}, {99.995, -100.0, 0.005}, {99.995, 100.0, 0.005}, {-100.005, 100.0, 0.005}};

      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructFromVertices(cubeOneVertices);
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructFromVertices(cubeTwoVertices);

      Point3d closestPointOnA = new Point3d();
      Point3d closestPointOnB = new Point3d();

      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);

      SimplexPolytope simplex = detector.getSimplex();
      int numberOfPointsOnSimplex = simplex.getNumberOfPoints();
      assertEquals(4, numberOfPointsOnSimplex);

      expandingPolytopeAlgorithm.setPolytopes(simplex, cubeOne, cubeTwo);
      Vector3d separatingDistanceVector = new Vector3d();
      expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

      assertEquals(1.469517381122009E-5, separatingDistanceVector.length(), 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(-0.6654362950758336, 0.0120271553579471, 0.00498530482618878), closestPointOnA, 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(-0.6654362950758336, 0.0120271553579471, 0.005), closestPointOnB, 1e-7);
   }

   private ConvexPolytope generateRandomCube(Random random)
   {
      double halfLengthX = RandomTools.generateRandomDouble(random, 0.10, 2.0);
      double halfWidthY = RandomTools.generateRandomDouble(random, 0.10, 0.5);
      double halfHeightZ = RandomTools.generateRandomDouble(random, 0.10, 1.0);

      return ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
   }

   private void translateObject(ConvexPolytope polytope, Vector3d translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(translation);
      polytope.applyTransform(transform);
   }

   private void rotateObject(ConvexPolytope polytope, double rotX, double rotY, double rotZ)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEulerAndZeroTranslation(rotX, rotY, rotZ);
      polytope.applyTransform(transform);
   }

}
