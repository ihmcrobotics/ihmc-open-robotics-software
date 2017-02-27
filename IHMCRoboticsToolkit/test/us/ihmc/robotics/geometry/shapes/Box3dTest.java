package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.shapes.Box3d.FaceName;
import us.ihmc.robotics.math.Epsilons;
import us.ihmc.robotics.random.RandomGeometry;

public class Box3dTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.9)
	@Test(timeout = 30000)
   public void testCommonShape3dFunctionality()
   {
      Shape3dTestHelper testHelper = new Shape3dTestHelper();
      Random random = new Random(1776L);
      
      int numberOfShapes = 1000;
      int numberOfPoints = 1000;
      
      for (int i=0; i<numberOfShapes; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         double length = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double width = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double height = RandomNumbers.nextDouble(random, 0.01, 10.0);
         Box3d box3d = new Box3d(transform, length, width, height);

         try
         {
            testHelper.runSimpleTests(box3d, random, numberOfPoints);
         }
         catch (AssertionError e)
         {
            e.printStackTrace();
            throw new AssertionError(e.getMessage());
         }
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSet()
   {
      Random random = new Random(1234L);
      Box3d box1 = createRandomBox(random);
      box1.getFace(FaceName.MINUSX);    // so that dirtyBit is false
      Box3d box2 = new Box3d();

      assertEverythingDifferent(box1, box2, 1e-14);
      box2.set(box1);
      assertBoxEquals(box1, box2, 1e-14);

      // make sure we're not copying references:
      box1.setFromTransform(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
      box1.setDimensions(random.nextDouble(), random.nextDouble(), random.nextDouble());
      assertEverythingDifferent(box1, box2, 1e-14);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyConstructor()
   {
      Random random = new Random(12434L);
      Box3d box1 = createRandomBox(random);
      Box3d box2 = new Box3d(box1);

      assertBoxEquals(box1, box2, 1e-14);

      // make sure we're not copying references:
      box1.setFromTransform(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
      box1.setDimensions(random.nextDouble(), random.nextDouble(), random.nextDouble());
      assertEverythingDifferent(box1, box2, 1e-14);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructors()
   {
      Random random = new Random(2345L);
      RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

      double length = random.nextDouble();
      double width = random.nextDouble();
      double height = random.nextDouble();

      Box3d box1 = new Box3d(transform, length, width, height);
      Box3d box2 = new Box3d(transform, new double[] {length, width, height});
      Box3d box3 = new Box3d(length, width, height);
      box3.setFromTransform(transform);

      assertBoxEquals(box1, box2, 0.0);
      assertBoxEquals(box1, box3, 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetTransform3DAndGetters()
   {
      Random random = new Random(351235L);
      Box3d box = new Box3d();
      RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      box.setFromTransform(transform);

      RigidBodyTransform transformBack = new RigidBodyTransform();
      box.getPose(transformBack);
      assertTrue(transform.epsilonEquals(transformBack, Epsilons.ONE_TRILLIONTH));
      assertTrue(transform.epsilonEquals(box.getTransformCopy(), Epsilons.ONE_TRILLIONTH));

      RotationMatrix matrix = new RotationMatrix();
      Vector3D vector = new Vector3D();
      transform.get(matrix, vector);

      RotationMatrix matrixBack = new RotationMatrix();
      Point3D pointBack = new Point3D();

      box.getRotation(matrixBack);
      assertTrue(matrix.epsilonEquals(matrixBack, Epsilons.ONE_TRILLIONTH));
      assertTrue(matrix.epsilonEquals(box.getRotationCopy(), Epsilons.ONE_TRILLIONTH));

      box.getCenter(pointBack);
      assertTrue(vector.epsilonEquals(pointBack, Epsilons.ONE_TRILLIONTH));
      assertTrue(vector.epsilonEquals(box.getCenterCopy(), Epsilons.ONE_TRILLIONTH));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testScale()
   {
      Box3d box3d = new Box3d();
      Plane3d xFace = box3d.getFace(FaceName.PLUSX);
      Plane3d yFace = box3d.getFace(FaceName.PLUSY);
      Plane3d zFace = box3d.getFace(FaceName.PLUSZ);
      Plane3d xFaceMinus = box3d.getFace(FaceName.MINUSX);
      Plane3d yFaceMinus = box3d.getFace(FaceName.MINUSY);
      Plane3d zFaceMinus = box3d.getFace(FaceName.MINUSZ);
      
      Plane3d expectedXFace = new Plane3d(new Point3D(0.5, 0.0, 0.0), new Vector3D(1.0, 0.0, 0.0));
      Plane3d expectedYFace = new Plane3d(new Point3D(0.0, 0.5, 0.0), new Vector3D(0.0, 1.0, 0.0));
      Plane3d expectedZFace = new Plane3d(new Point3D(0.0, 0.0, 0.5), new Vector3D(0.0, 0.0, 1.0));
      Plane3d expectedXFaceMinus = new Plane3d(new Point3D(-0.5, 0.0, 0.0), new Vector3D(-1.0, 0.0, 0.0));
      Plane3d expectedYFaceMinus = new Plane3d(new Point3D(0.0, -0.5, 0.0), new Vector3D(0.0, -1.0, 0.0));
      Plane3d expectedZFaceMinus = new Plane3d(new Point3D(0.0, 0.0, -0.5), new Vector3D(0.0, 0.0, -1.0));
      
      double epsilon = 1e-14;
      assertTrue(xFace.epsilonEquals(expectedXFace, epsilon));
      assertTrue(yFace.epsilonEquals(expectedYFace, epsilon));
      assertTrue(zFace.epsilonEquals(expectedZFace, epsilon));
      assertTrue(xFaceMinus.epsilonEquals(expectedXFaceMinus, epsilon));
      assertTrue(yFaceMinus.epsilonEquals(expectedYFaceMinus, epsilon));
      assertTrue(zFaceMinus.epsilonEquals(expectedZFaceMinus, epsilon));
      
      box3d.scale(2.0);
      xFace = box3d.getFace(FaceName.PLUSX);
      yFace = box3d.getFace(FaceName.PLUSY);
      zFace = box3d.getFace(FaceName.PLUSZ);
      xFaceMinus = box3d.getFace(FaceName.MINUSX);
      yFaceMinus = box3d.getFace(FaceName.MINUSY);
      zFaceMinus = box3d.getFace(FaceName.MINUSZ);
      
      expectedXFace = new Plane3d(new Point3D(1.0, 0.0, 0.0), new Vector3D(2.0, 0.0, 0.0));
      expectedYFace = new Plane3d(new Point3D(0.0, 1.0, 0.0), new Vector3D(0.0, 2.0, 0.0));
      expectedZFace = new Plane3d(new Point3D(0.0, 0.0, 1.0), new Vector3D(0.0, 0.0, 2.0));
      expectedXFaceMinus = new Plane3d(new Point3D(-1.0, 0.0, 0.0), new Vector3D(-2.0, 0.0, 0.0));
      expectedYFaceMinus = new Plane3d(new Point3D(0.0, -1.0, 0.0), new Vector3D(0.0, -2.0, 0.0));
      expectedZFaceMinus = new Plane3d(new Point3D(0.0, 0.0, -1.0), new Vector3D(0.0, 0.0, -2.0));
      
      assertTrue(xFace.epsilonEquals(expectedXFace, epsilon));
      assertTrue(yFace.epsilonEquals(expectedYFace, epsilon));
      assertTrue(zFace.epsilonEquals(expectedZFace, epsilon));
      assertTrue(xFaceMinus.epsilonEquals(expectedXFaceMinus, epsilon));
      assertTrue(yFaceMinus.epsilonEquals(expectedYFaceMinus, epsilon));
      assertTrue(zFaceMinus.epsilonEquals(expectedZFaceMinus, epsilon));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsInsideOrOnSurfaceConvexCombinationOfVertices()
   {
      Random random = new Random(123234L);
      Box3d box = createRandomBox(random);

      Point3D[] vertices = new Point3D[Box3d.NUM_VERTICES];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.computeVertices(vertices);

      int nTests = 300;
      for (int i = 0; i < nTests; i++)
      {
         Point3D pointToTest = getRandomConvexCombination(random, vertices);

         assertTrue(box.isInsideOrOnSurface(pointToTest));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testVerticesProjection()
   {
      Random random = new Random(123234L);
      Box3d box = createRandomBox(random);

      Point3D[] vertices = new Point3D[Box3d.NUM_VERTICES];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.computeVertices(vertices);

      for (Point3D vertex : vertices)
      {
         Point3D projectedVertex = new Point3D(vertex);
         box.orthogonalProjection(projectedVertex);
         EuclidCoreTestTools.assertTuple3DEquals(vertex, projectedVertex, 1e-14);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsInsideOrOnSurfaceVertices()
   {
      Random random = new Random(123234L);
      Box3d box = createRandomBox(random);

      Point3D[] vertices = new Point3D[Box3d.NUM_VERTICES];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.computeVertices(vertices);

      for (Point3D vertex : vertices)
      {
         assertTrue(box.isInsideOrOnSurface(vertex, 1e-14));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testProjectionToVertices()
   {
      Random random = new Random(123234L);

      int nTests = 300;
      for (int testIndex = 0; testIndex < nTests; testIndex++)
      {
         Box3d box = createRandomBox(random);

         Point3D[] vertices = new Point3D[Box3d.NUM_VERTICES];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3D();
         }

         double epsilon = Epsilons.ONE_TRILLIONTH;
         double maxScale = 2.0;
         box.computeVertices(vertices);
         Point3D center = box.getCenterCopy();
         for (Point3D vertex : vertices)
         {
            Vector3D offset = new Vector3D(vertex);
            offset.sub(center);
            offset.normalize();
            offset.scale(RandomNumbers.nextDouble(random, epsilon, maxScale));
            Point3D testPoint = new Point3D(vertex);
            testPoint.add(offset);
            assertFalse(box.isInsideOrOnSurface(testPoint, epsilon));

            Point3D projection = new Point3D(testPoint);
            box.orthogonalProjection(projection);
            EuclidCoreTestTools.assertTuple3DEquals(vertex, projection, epsilon);
            assertTrue(box.isInsideOrOnSurface(projection, epsilon));
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeVertices()
   {
      Random random = new Random(6234L);
      double length = random.nextDouble();
      double width = random.nextDouble();
      double height = random.nextDouble();
      RigidBodyTransform configuration = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      Box3d box3d = new Box3d(configuration, length, width, height);

      ArrayList<Point3D> expectedVertices = new ArrayList<Point3D>(Box3d.NUM_VERTICES);
      expectedVertices.add(new Point3D(length / 2.0, width / 2.0, height / 2.0));
      expectedVertices.add(new Point3D(length / 2.0, width / 2.0, -height / 2.0));
      expectedVertices.add(new Point3D(length / 2.0, -width / 2.0, height / 2.0));
      expectedVertices.add(new Point3D(length / 2.0, -width / 2.0, -height / 2.0));
      expectedVertices.add(new Point3D(-length / 2.0, width / 2.0, height / 2.0));
      expectedVertices.add(new Point3D(-length / 2.0, width / 2.0, -height / 2.0));
      expectedVertices.add(new Point3D(-length / 2.0, -width / 2.0, height / 2.0));
      expectedVertices.add(new Point3D(-length / 2.0, -width / 2.0, -height / 2.0));

      for (Point3D point3d : expectedVertices)
      {
         configuration.transform(point3d);
      }

      Point3D[] vertices = new Point3D[Box3d.NUM_VERTICES];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box3d.computeVertices(vertices);

      double epsilon = Epsilons.ONE_TRILLIONTH;
      for (Point3D vertex : vertices)
      {
         Iterator<Point3D> iterator = expectedVertices.iterator();
         while (iterator.hasNext())
         {
            Point3D expectedVertex = iterator.next();
            if (expectedVertex.epsilonEquals(vertex, epsilon))
               iterator.remove();
         }
      }

      assertEquals(0, expectedVertices.size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFacesOrthogonal()
   {
      int nTests = 300;
      Random random = new Random(61346L);

      ArrayList<ImmutablePair<FaceName, FaceName>> faces = new ArrayList<ImmutablePair<FaceName, FaceName>>();
      faces.add(new ImmutablePair<FaceName, FaceName>(FaceName.PLUSX, FaceName.MINUSX));
      faces.add(new ImmutablePair<FaceName, FaceName>(FaceName.PLUSY, FaceName.MINUSY));
      faces.add(new ImmutablePair<FaceName, FaceName>(FaceName.PLUSZ, FaceName.MINUSZ));

      double epsilon = Epsilons.ONE_TRILLIONTH;
      for (int i = 0; i < nTests; i++)
      {
         Box3d box = createRandomBox(random);
         Collections.shuffle(faces);

         for (ImmutablePair<FaceName, FaceName> pair : faces)
         {
            Vector3D firstNormal = box.getFace(pair.getLeft()).getNormalCopy();
            Vector3D secondNormal = box.getFace(pair.getRight()).getNormalCopy();
            assertEquals(-1.0, firstNormal.dot(secondNormal), epsilon);

            for (ImmutablePair<FaceName, FaceName> otherPair : faces)
            {
               if (pair != otherPair)
               {
                  Vector3D otherNormal = box.getFace(otherPair.getLeft()).getNormalCopy();
                  assertEquals(0.0, firstNormal.dot(otherNormal), epsilon);

               }
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRotation()
   {
      int nTests = 300;
      Random random = new Random(1345L);

      ArrayList<ImmutablePair<FaceName, Integer>> faces = new ArrayList<ImmutablePair<FaceName,Integer>>();
      faces.add(new ImmutablePair<FaceName, Integer>(FaceName.PLUSX, 0));
      faces.add(new ImmutablePair<FaceName, Integer>(FaceName.PLUSY, 1));
      faces.add(new ImmutablePair<FaceName, Integer>(FaceName.PLUSZ, 2));

      Vector3D center = RandomGeometry.nextVector3D(random);
      double length = random.nextDouble();
      double width = random.nextDouble();
      double height = random.nextDouble();

      Box3d box = new Box3d(length, width, height);
      box.setTranslation(new Point3D(center));

      for (int i = 0; i < nTests; i++)
      {
         RotationMatrix rotation = new RotationMatrix();
         rotation.set(RandomGeometry.nextAxisAngle(random));
         box.setRotation(rotation);
         assertEquals(length, box.getLength(), Epsilons.ONE_TRILLIONTH);
         assertEquals(width, box.getWidth(), Epsilons.ONE_TRILLIONTH);
         assertEquals(height, box.getHeight(), Epsilons.ONE_TRILLIONTH);
         EuclidCoreTestTools.assertTuple3DEquals(center, box.getCenterCopy(), Epsilons.ONE_TRILLIONTH);
         EuclidCoreTestTools.assertMatrix3DEquals("", rotation, box.getRotationCopy(), Epsilons.ONE_TRILLIONTH);
         
         Collections.shuffle(faces);
         for (ImmutablePair<FaceName, Integer> face : faces)
         {
            Plane3d plane = box.getFace(face.getLeft());
            Vector3D normal = plane.getNormalCopy();
            
            Vector3D rotationMatrixColumn = new Vector3D();
            rotation.getColumn(face.getRight(), rotationMatrixColumn);
            EuclidCoreTestTools.assertTuple3DEquals(rotationMatrixColumn, normal, Epsilons.ONE_TRILLIONTH);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testDistanceProjectionAndClosestPointAndNormal()
   {
      Random random = new Random(123415L);
      int nBoxes = 10;
      int nTestsPerBox = 1000;
      for (int boxNumber = 0; boxNumber < nBoxes; boxNumber++)
      {
         Box3d box = createRandomBox(random);
         Box3d biggerBox = new Box3d(box);
         biggerBox.scale(2.0);

         Point3D[] vertices = new Point3D[Box3d.NUM_VERTICES];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3D();
         }
         biggerBox.computeVertices(vertices);

         for (int testNumber = 0; testNumber < nTestsPerBox; testNumber++)
         {
            Point3D point = getRandomConvexCombination(random, vertices);
            Point3D projectedPoint = new Point3D(point);
            box.orthogonalProjection(projectedPoint);
            
            Point3D closestPoint = new Point3D();
            Vector3D normal = new Vector3D();
            box.checkIfInside(point, closestPoint, normal);

            // check distance stuff:
            double epsilon = 1e-14;
            assertEquals(box.distance(closestPoint), 0.0, epsilon);
            assertTrue(box.isInsideOrOnSurface(closestPoint, epsilon));
            assertTrue(box.isInsideOrOnSurface(projectedPoint, epsilon));
            if (box.isInsideOrOnSurface(point))
            {
               EuclidCoreTestTools.assertTuple3DEquals(point, projectedPoint, 1e-10);
            }
            else
            {
               assertEquals(point.distance(projectedPoint), box.distance(point), epsilon);
               EuclidCoreTestTools.assertTuple3DEquals(projectedPoint, closestPoint, epsilon);
            }
            
            // create 3 points that are close to point and check whether a numerical normal to a plane through those points equals the returned normal:
            double delta = 1e-5;

            Point3D point2 = new Point3D(point);
            point2.add(RandomGeometry.nextVector3D(random, delta));           
            Point3D closestPoint2 = new Point3D();
            Vector3D normal2 = new Vector3D();
            box.checkIfInside(point2, closestPoint2, normal2);
            
            Point3D point3 = new Point3D(point);
            point3.add(RandomGeometry.nextVector3D(random, delta));
            Point3D closestPoint3 = new Point3D();
            Vector3D normal3 = new Vector3D();
            box.checkIfInside(point3, closestPoint3, normal3);
            
            boolean pointsDistinct = closestPoint2.distance(closestPoint) > epsilon && closestPoint3.distance(closestPoint) > epsilon && closestPoint3.distance(closestPoint2) > epsilon;
            if (pointsDistinct)
            {
               boolean normalsEqual = normal.epsilonEquals(normal2, epsilon) && normal.epsilonEquals(normal3, epsilon);
               if (normalsEqual)
               {
                  Vector3D vector1 = new Vector3D();
                  vector1.sub(closestPoint2, closestPoint);
                  
                  Vector3D vector2 = new Vector3D();
                  vector2.sub(closestPoint3, closestPoint);
                  
                  Vector3D normalFromCrossProduct = new Vector3D();
                  normalFromCrossProduct.cross(vector1, vector2);
                  normalFromCrossProduct.normalize();

                  assertEquals(1.0, Math.abs(normalFromCrossProduct.dot(normal)), 1e-8);
               }
            }
         }
      }
   }
	
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testApplyTransform()
   {
      Random random = new Random(562346L);
      int nBoxes = 100;
      int nTestsPerBox = 1000;
      for (int boxNumber = 0; boxNumber < nBoxes; boxNumber++)
      {
         Box3d box = createRandomBox(random);
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         Box3d boxTransformed = new Box3d(box);
         boxTransformed.applyTransform(transform);
         Box3d biggerBox = new Box3d(box);
         biggerBox.scale(2.0);

         Point3D[] vertices = new Point3D[Box3d.NUM_VERTICES];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3D();
         }
         biggerBox.computeVertices(vertices);
      
         for (int testNumber = 0; testNumber < nTestsPerBox; testNumber++)
         {
            Point3D point = getRandomConvexCombination(random, vertices);
            Point3D pointTransformed = new Point3D(point);
            box.transformToLocal(pointTransformed);
            boxTransformed.transformToWorld(pointTransformed);
            
            assertEquals(box.isInsideOrOnSurface(point), boxTransformed.isInsideOrOnSurface(pointTransformed));            
         }
      }
   }
	
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testApplyTransform2()
   {
      Box3d box3d = new Box3d();
      RigidBodyTransform transform = new RigidBodyTransform();
      Point3D point = new Point3D();
      point.set(1.0, 1.0, 1.0);
      transform.setTranslation(point);
      box3d.applyTransform(transform);
      
      box3d.getCenter(point);
      Point3D expectedPosition = new Point3D(1.0, 1.0, 1.0);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPosition, point, Epsilons.ONE_TRILLIONTH);
      
      Quaternion quat = new Quaternion();
      quat.setYawPitchRoll(1.0, 1.0, 1.0);
      Quaternion quat2 = new Quaternion(quat);
      transform.setRotationAndZeroTranslation(quat);
      
      box3d.applyTransform(transform);
      
      box3d.getPosition(point);
      box3d.getOrientation(quat);
      expectedPosition.applyTransform(transform);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPosition, point, Epsilons.ONE_TRILLIONTH);
      EuclidCoreTestTools.assertQuaternionEquals(quat2, quat, Epsilons.ONE_TRILLIONTH);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetYawPitchRoll()
   {
      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         Random random = new Random(562346L);
         Box3d box = new Box3d();
         
         double yaw = random.nextDouble();
         double pitch = random.nextDouble();
         double roll = random.nextDouble();
         box.setYawPitchRoll(yaw, pitch, roll);
         
         RotationMatrix rotation = new RotationMatrix();
         box.getRotation(rotation);
         
         double epsilon = 1e-14;
         assertEquals(yaw, rotation.getYaw(), epsilon);
         assertEquals(pitch, rotation.getPitch(), epsilon);
         assertEquals(roll, rotation.getRoll(), epsilon);
      }
   }

   private static Box3d createRandomBox(Random random)
   {
      RigidBodyTransform configuration = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      double lengthX = random.nextDouble();
      double widthY = random.nextDouble();
      double heightZ = random.nextDouble();

      return new Box3d(configuration, lengthX, widthY, heightZ);
   }

   private static void assertEverythingDifferent(Box3d box1, Box3d box2, double epsilon)
   {
      assertFalse(box1.getTransformCopy().epsilonEquals(box2.getTransformCopy(), epsilon));

      for (Direction direction : Direction.values())
      {
         assertFalse(MathTools.epsilonEquals(box1.getDimension(direction), box2.getDimension(direction), epsilon));
      }
   }

   private static void assertBoxEquals(Box3d box1, Box3d box2, double epsilon)
   {
      assertTrue(box1.getTransformCopy().epsilonEquals(box2.getTransformCopy(), epsilon));

      for (Direction direction : Direction.values())
      {
         assertEquals(box1.getDimension(direction), box2.getDimension(direction), epsilon);
      }
   }
   
   private static Point3D getRandomConvexCombination(Random random, Point3D[] vertices)
   {
      double[] weightings = new double[vertices.length];
      for (int j = 0; j < weightings.length; j++)
      {
         weightings[j] = random.nextDouble();
      }

      double sum = MathTools.sumDoubles(weightings);
      for (int j = 0; j < weightings.length; j++)
      {
         weightings[j] /= sum;
      }

      Point3D pointToTest = new Point3D();
      for (int j = 0; j < weightings.length; j++)
      {
         Point3D tempPoint = new Point3D(vertices[j]);
         tempPoint.scale(weightings[j]);
         pointToTest.add(tempPoint);
      }
      return pointToTest;
   }
}
