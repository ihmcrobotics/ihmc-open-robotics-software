package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;

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
      ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructSinglePointPolytope(new Point3D());

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

      Point3D closestPointOnA = new Point3D();
      Point3D closestPointOnB = new Point3D();

      boolean areColliding = detector.arePolytopesColliding(polytopeOne, polytopeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);

      SimplexPolytope simplex = detector.getSimplex();
      int numberOfPointsOnSimplex = simplex.getNumberOfPoints();
      assertEquals(4, numberOfPointsOnSimplex);

      expandingPolytopeAlgorithm.setPolytopes(simplex, polytopeOne, polytopeTwo);
      Vector3D separatingDistanceVector = new Vector3D();
      expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, -0.2), separatingDistanceVector, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, -0.15), closestPointOnA, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 0.05), closestPointOnB, 1e-7);
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

      translateObject(cubeOne, new Vector3D(1.4, 2.5, 3.8));
      translateObject(cubeTwo, new Vector3D(1.0, 2.0, 3.0));

      Point3D closestPointOnA = new Point3D();
      Point3D closestPointOnB = new Point3D();

      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);

      SimplexPolytope simplex = detector.getSimplex();
      int numberOfPointsOnSimplex = simplex.getNumberOfPoints();
      assertEquals(4, numberOfPointsOnSimplex);

      expandingPolytopeAlgorithm.setPolytopes(simplex, cubeOne, cubeTwo);
      Vector3D separatingDistanceVector = new Vector3D();
      expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, -0.2), separatingDistanceVector, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.7, 2.75, 3.8), closestPointOnA, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.7, 2.75, 4.0), closestPointOnB, 1e-7);
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

         rotateObject(cubeOne, RandomNumbers.nextDouble(random, Math.PI), RandomNumbers.nextDouble(random, Math.PI),
               RandomNumbers.nextDouble(random, Math.PI));
         rotateObject(cubeTwo, RandomNumbers.nextDouble(random, Math.PI), RandomNumbers.nextDouble(random, Math.PI),
               RandomNumbers.nextDouble(random, Math.PI));

         translateObject(cubeOne, RandomGeometry.nextVector3D(random, new Vector3D(5.0, -7.0, 10.0), new Vector3D(7.0, -5.0, 12.0)));
         translateObject(cubeTwo, RandomGeometry.nextVector3D(random, new Vector3D(5.0, -7.0, 10.0), new Vector3D(7.0, -5.0, 12.0)));

         Point3D closestPointOnA = new Point3D();
         Point3D closestPointOnB = new Point3D();

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
               Vector3D separatingDistanceVector = new Vector3D();
               expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

               assertEquals(separatingDistanceVector.length(), closestPointOnA.distance(closestPointOnB), 1e-7);
            }
         }
      }

      assertTrue("numberOfCollisions = " + numberOfCollisions, numberOfCollisions > 500);
   }

   @Test
   public void testExtensivelyWithPolytopes()
   {
      Random random = new Random(1886L);

      double epsilonRelative = 1e-5;
      ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
      ExpandingPolytopeAlgorithmAssertListener expandingPolytopeListener = new ExpandingPolytopeAlgorithmAssertListener();
      expandingPolytopeAlgorithm.setExpandingPolytopeAlgorithmListener(expandingPolytopeListener);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      int numberOfTests = 10000;

      int numberOfCollisions = 0;
      for (int i = 0; i < numberOfTests; i++)
      {
         double xyzBoundary = 0.0;

         double radiusOne = RandomNumbers.nextDouble(random, 0.5, 2.0);
         double radiusTwo = RandomNumbers.nextDouble(random, 0.5, 2.0);

         int numberOfPointsOne = RandomNumbers.nextInt(random, 4, 20);
         int numberOfPointsTwo = RandomNumbers.nextInt(random, 4, 20);

         ConvexPolytope polytopeOne = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPointsOne, radiusOne, xyzBoundary);
         ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPointsTwo, radiusTwo, xyzBoundary);

         rotateObject(polytopeOne, RandomNumbers.nextDouble(random, Math.PI), RandomNumbers.nextDouble(random, Math.PI),
               RandomNumbers.nextDouble(random, Math.PI));
         rotateObject(polytopeTwo, RandomNumbers.nextDouble(random, Math.PI), RandomNumbers.nextDouble(random, Math.PI),
               RandomNumbers.nextDouble(random, Math.PI));

         translateObject(polytopeOne, RandomGeometry.nextVector3D(random, new Vector3D(4.5, -10.0, 8.5), new Vector3D(6.5, -7.0, 11.5)));
         translateObject(polytopeTwo, RandomGeometry.nextVector3D(random, new Vector3D(4.5, -10.0, 8.5), new Vector3D(6.5, -7.0, 11.5)));

         Point3D closestPointOnA = new Point3D();
         Point3D closestPointOnB = new Point3D();

         boolean areColliding = detector.arePolytopesColliding(polytopeOne, polytopeTwo, closestPointOnA, closestPointOnB);

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
               expandingPolytopeAlgorithm.setPolytopes(simplex, polytopeOne, polytopeTwo);
               Vector3D separatingDistanceVector = new Vector3D();
               expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

               assertEquals(separatingDistanceVector.length(), closestPointOnA.distance(closestPointOnB), 1e-7);

               //             Move halfway in the separating distance direction and make sure they are not colliding...

               RigidBodyTransform moveInSeparatingDirection = new RigidBodyTransform();
               Vector3D separatingDistanceVectorPlusSome = new Vector3D(separatingDistanceVector);
               separatingDistanceVectorPlusSome.scale(0.5);
               moveInSeparatingDirection.setTranslation(separatingDistanceVectorPlusSome);
               polytopeTwo.applyTransform(moveInSeparatingDirection);

               areColliding = detector.arePolytopesColliding(polytopeOne, polytopeTwo, closestPointOnA, closestPointOnB);
               assertTrue(areColliding);
               double newDistance = closestPointOnA.distance(closestPointOnB);

               // Move in the separating distance direction and make sure they are not colliding...

               if (separatingDistanceVector.length() > 0.001)
               {
                  moveInSeparatingDirection = new RigidBodyTransform();
                  separatingDistanceVectorPlusSome = new Vector3D(separatingDistanceVector);
                  separatingDistanceVectorPlusSome.scale(0.6);
                  moveInSeparatingDirection.setTranslation(separatingDistanceVectorPlusSome);
                  polytopeTwo.applyTransform(moveInSeparatingDirection);

                  areColliding = detector.arePolytopesColliding(polytopeOne, polytopeTwo, closestPointOnA, closestPointOnB);
                  assertFalse(areColliding);
                  newDistance = closestPointOnA.distance(closestPointOnB);
                  assertEquals(0.1 * separatingDistanceVector.length(), newDistance, 1e-7);
               }
            }
         }
      }

      assertTrue("numberOfCollisions = " + numberOfCollisions, numberOfCollisions > 500);
   }

   @Test
   public void testExtensivelyWithCylinders()
   {
      Random random = new Random(1886L);

      double epsilonRelative = 1e-5;
      ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
      ExpandingPolytopeAlgorithmAssertListener expandingPolytopeListener = new ExpandingPolytopeAlgorithmAssertListener();
      expandingPolytopeAlgorithm.setExpandingPolytopeAlgorithmListener(expandingPolytopeListener);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      int numberOfTests = 10000;

      int numberOfCollisions = 0;
      for (int i = 0; i < numberOfTests; i++)
      {
         double xyzBoundary = 0.0;

         double radiusOne = RandomNumbers.nextDouble(random, 0.5, 2.0);
         double radiusTwo = RandomNumbers.nextDouble(random, 0.5, 2.0);
         double heightTwo = RandomNumbers.nextDouble(random, 0.5, 2.0);

         int numberOfPointsOne = RandomNumbers.nextInt(random, 4, 20);

         ConvexPolytope polytopeOne = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPointsOne, radiusOne, xyzBoundary);
         CylinderSupportingVertexHolder cylinderTwo = new CylinderSupportingVertexHolder(radiusTwo, heightTwo);

         rotateObject(polytopeOne, RandomNumbers.nextDouble(random, Math.PI), RandomNumbers.nextDouble(random, Math.PI),
               RandomNumbers.nextDouble(random, Math.PI));
         rotateObject(cylinderTwo, RandomNumbers.nextDouble(random, Math.PI), RandomNumbers.nextDouble(random, Math.PI),
               RandomNumbers.nextDouble(random, Math.PI));

         translateObject(polytopeOne, RandomGeometry.nextVector3D(random, new Vector3D(4.5, -10.0, 8.5), new Vector3D(6.5, -7.0, 11.5)));
         translateObject(cylinderTwo, RandomGeometry.nextVector3D(random, new Vector3D(4.5, -10.0, 8.5), new Vector3D(6.5, -7.0, 11.5)));

         Point3D closestPointOnA = new Point3D();
         Point3D closestPointOnB = new Point3D();

         boolean areColliding = detector.arePolytopesColliding(polytopeOne, cylinderTwo, closestPointOnA, closestPointOnB);

//         System.out.println(polytopeOne);
//         System.out.println(cylinderTwo);
//         System.out.flush();

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
               expandingPolytopeAlgorithm.setPolytopes(simplex, polytopeOne, cylinderTwo);
               Vector3D separatingDistanceVector = new Vector3D();
               expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

               assertEquals(separatingDistanceVector.length(), closestPointOnA.distance(closestPointOnB), 1e-7);

               //             Move halfway in the separating distance direction and make sure they are not colliding...

               RigidBodyTransform moveInSeparatingDirection = new RigidBodyTransform();
               Vector3D separatingDistanceVectorPlusSome = new Vector3D(separatingDistanceVector);
               separatingDistanceVectorPlusSome.scale(0.5);
               moveInSeparatingDirection.setTranslation(separatingDistanceVectorPlusSome);
               cylinderTwo.applyTransform(moveInSeparatingDirection);

               areColliding = detector.arePolytopesColliding(polytopeOne, cylinderTwo, closestPointOnA, closestPointOnB);
               assertTrue(areColliding);
               double newDistance = closestPointOnA.distance(closestPointOnB);

               // Move in the separating distance direction and make sure they are not colliding...

               if (separatingDistanceVector.length() > 0.003)
               {
                  moveInSeparatingDirection = new RigidBodyTransform();
                  separatingDistanceVectorPlusSome = new Vector3D(separatingDistanceVector);
                  separatingDistanceVectorPlusSome.scale(0.6);
                  moveInSeparatingDirection.setTranslation(separatingDistanceVectorPlusSome);
                  cylinderTwo.applyTransform(moveInSeparatingDirection);

                  areColliding = detector.arePolytopesColliding(polytopeOne, cylinderTwo, closestPointOnA, closestPointOnB);
                  assertFalse(areColliding);
                  newDistance = closestPointOnA.distance(closestPointOnB);
                  assertEquals(0.1 * separatingDistanceVector.length(), newDistance, 2e-3);
               }
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

      Point3D closestPointOnA = new Point3D();
      Point3D closestPointOnB = new Point3D();

      boolean areColliding = detector.arePolytopesColliding(cubeOne, cubeTwo, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);

      SimplexPolytope simplex = detector.getSimplex();
      int numberOfPointsOnSimplex = simplex.getNumberOfPoints();
      assertEquals(4, numberOfPointsOnSimplex);

      expandingPolytopeAlgorithm.setPolytopes(simplex, cubeOne, cubeTwo);
      Vector3D separatingDistanceVector = new Vector3D();
      expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);

      assertEquals(1.469517381122009E-5, separatingDistanceVector.length(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-0.6654362950758336, 0.0120271553579471, 0.00498530482618878), closestPointOnA, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-0.6654362950758336, 0.0120271553579471, 0.005), closestPointOnB, 1e-7);
   }

   @Test
   public void testTroublesomeCylinder()
   {
      double epsilonRelative = 1e-5;
      ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
      ExpandingPolytopeAlgorithmAssertListener expandingPolytopeListener = new ExpandingPolytopeAlgorithmAssertListener();
      expandingPolytopeAlgorithm.setExpandingPolytopeAlgorithmListener(expandingPolytopeListener);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();
      GilbertJohnsonKeerthiCollisionDetectorAssertListener listener = new GilbertJohnsonKeerthiCollisionDetectorAssertListener();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      double[][] polytopeVertices = new double[][] {{4.7290109868252195, -9.48051959337279, 9.86066553142316},
            {4.992334394846338, -8.508417923723337, 11.746865167550746}, {4.909690197353282, -9.043958943334093, 11.732447362884571},
            {4.227668551037339, -8.116865776445794, 10.880447832028764}, {5.053783588777796, -9.708661418625793, 10.018042440299405},
            {6.191961614893392, -8.684064716610006, 11.01506200303345}};

      ConvexPolytope polytope = ConvexPolytopeConstructor.constructFromVertices(polytopeVertices);

      double radius = 1.4057784173328112;
      double height = 0.807422643821864;

      CylinderSupportingVertexHolder cylinder = new CylinderSupportingVertexHolder(radius, height);
      DenseMatrix64F matrix = new DenseMatrix64F(new double[][] {{-0.04233754, 0.45458570, 0.88969623, 5.84106795},
            {0.36801675, 0.83497813, -0.40911513, -8.76333957}, {-0.92885478, 0.31010218, -0.20264607, 9.64464579}, {0, 0, 0, 1}});

      RigidBodyTransform transform = new RigidBodyTransform(matrix);
      cylinder.setTransform(transform);

      Point3D closestPointOnA = new Point3D();
      Point3D closestPointOnB = new Point3D();

      boolean areColliding = detector.arePolytopesColliding(polytope, cylinder, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);

      SimplexPolytope simplex = detector.getSimplex();
      int numberOfPointsOnSimplex = simplex.getNumberOfPoints();
      assertEquals(4, numberOfPointsOnSimplex);

      expandingPolytopeAlgorithm.setPolytopes(simplex, polytope, cylinder);
      Vector3D separatingDistanceVector = new Vector3D();
      expandingPolytopeAlgorithm.computeExpandedPolytope(separatingDistanceVector, closestPointOnA, closestPointOnB);
  
     
      assertEquals(0.38364002935502967, separatingDistanceVector.length(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(6.0695755857656915, -8.794237570890989, 10.907854463926327), closestPointOnA, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(5.770928840941696, -8.650471584361217, 11.101041251897659), closestPointOnB, 1e-7);
      
      assertEquals(separatingDistanceVector.length(), closestPointOnA.distance(closestPointOnB), 1e-7);

      // Move halfway in the separating distance direction and make sure they are not colliding...

      RigidBodyTransform moveInSeparatingDirection = new RigidBodyTransform();
      Vector3D separatingDistanceVectorPlusSome = new Vector3D(separatingDistanceVector);
      separatingDistanceVectorPlusSome.scale(0.5);
      moveInSeparatingDirection.setTranslation(separatingDistanceVectorPlusSome);
      cylinder.applyTransform(moveInSeparatingDirection);

      areColliding = detector.arePolytopesColliding(polytope, cylinder, closestPointOnA, closestPointOnB);
      assertTrue(areColliding);
      double newDistance = closestPointOnA.distance(closestPointOnB);

      // Move in the separating distance direction and make sure they are not colliding...

      if (separatingDistanceVector.length() > 0.003)
      {
         moveInSeparatingDirection = new RigidBodyTransform();
         separatingDistanceVectorPlusSome = new Vector3D(separatingDistanceVector);
         separatingDistanceVectorPlusSome.scale(0.6);
         moveInSeparatingDirection.setTranslation(separatingDistanceVectorPlusSome);
         cylinder.applyTransform(moveInSeparatingDirection);

         areColliding = detector.arePolytopesColliding(polytope, cylinder, closestPointOnA, closestPointOnB);
         assertFalse(areColliding);
         newDistance = closestPointOnA.distance(closestPointOnB);
         assertEquals(0.1 * separatingDistanceVector.length(), newDistance, 2e-3);
      }
   }

   private ConvexPolytope generateRandomCube(Random random)
   {
      double halfLengthX = RandomNumbers.nextDouble(random, 0.10, 2.0);
      double halfWidthY = RandomNumbers.nextDouble(random, 0.10, 0.5);
      double halfHeightZ = RandomNumbers.nextDouble(random, 0.10, 1.0);

      return ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
   }

   private void translateObject(ConvexPolytope polytope, Vector3D translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(translation);
      polytope.applyTransform(transform);
   }

   private void translateObject(CylinderSupportingVertexHolder cylinder, Vector3D translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(translation);
      cylinder.applyTransform(transform);
   }

   private void rotateObject(ConvexPolytope polytope, double rotX, double rotY, double rotZ)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEulerAndZeroTranslation(rotX, rotY, rotZ);
      polytope.applyTransform(transform);
   }

   private void rotateObject(CylinderSupportingVertexHolder cylinder, double rotX, double rotY, double rotZ)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEulerAndZeroTranslation(rotX, rotY, rotZ);
      cylinder.applyTransform(transform);
   }

}
