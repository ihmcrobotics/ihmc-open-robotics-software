package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomTools;

public class Ramp3dTest
{
	private static final boolean DEBUG = false;

	/**
	 * Ramp3d needs a little more work and the tests improve. It's hard to do really good surface normal tests at the corners.
	 */
	@ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout = 502)
   public void testCommonShape3dFunctionality()
   {
      Shape3dTestHelper testHelper = new Shape3dTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         double length = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         double width = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         double height = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         Ramp3d box3d = new Ramp3d(transform, width, length, height);
         testHelper.runSimpleTests(box3d, random, numberOfPoints);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testExampleUsage()
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(Math.PI / 2.0);
      transform.setTranslation(new Vector3D(2.0, 0.0, 3.0));

      // example use
      Ramp3d ramp3d = new Ramp3d(transform, 1.0, 1.0, 1.0);
      Point3D pointToCheck = new Point3D(2.0, 0.0, 4.0);
      assertFalse(ramp3d.isInsideOrOnSurface(pointToCheck));
      assertEquals(Math.toRadians(45.0), ramp3d.getRampIncline(), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetAndSet()
   {
      Random random = new Random(1024L);
      Ramp3d ramp1 = createRandomRamp(random);

      Ramp3d ramp2 = new Ramp3d(1.0, 1.0, 1.0);
      ramp2.setWidth(ramp1.getWidth());
      ramp2.setLength(ramp1.getLength());
      ramp2.setHeight(ramp1.getHeight());
      ramp2.setTransform(ramp1.getTransformUnsafe());

      assertTrue(ramp1.getWidth() == ramp2.getWidth());
      assertTrue(ramp1.getLength() == ramp2.getLength());
      assertTrue(ramp1.getHeight() == ramp2.getHeight());
      assertEquals(ramp1.getTransformUnsafe(), ramp2.getTransformUnsafe());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSurfaceNormal()
   {
      Ramp3d ramp = new Ramp3d(1.0, 1.0, 1.0);
      Vector3D surfaceNormal = new Vector3D();
      ramp.getSurfaceNormal(surfaceNormal);
      assertEquals("not equal", surfaceNormal.getX(), -1.0 / Math.sqrt(2.0), 1e-14);
      assertEquals("not equal", surfaceNormal.getY(), 0.0, 1e-14);
      assertEquals("not equal", surfaceNormal.getZ(), 1.0 / Math.sqrt(2.0), 1e-14);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleOrthogonalProjection()
   {
      Ramp3d ramp3d = new Ramp3d(1.0, 1.0, 1.0);
      Point3D pointToProject = new Point3D(0.0, 0.0, 1.0);
      ramp3d.orthogonalProjection(pointToProject);
      assertEquals(pointToProject.getX(), 0.5, 1e-14);
      assertEquals(pointToProject.getY(), 0.0, 1e-14);
      assertEquals(pointToProject.getZ(), 0.5, 1e-14);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(0.5 * Math.PI);
      ramp3d.applyTransform(transform);
      pointToProject.set(0.0, 0.0, 1.0);
      ramp3d.orthogonalProjection(pointToProject);
      assertEquals(pointToProject.getX(), 0.0, 1e-14);
      assertEquals(pointToProject.getY(), 0.5, 1e-14);
      assertEquals(pointToProject.getZ(), 0.5, 1e-14);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimplePointOutside()
   {
      Ramp3d ramp3d = new Ramp3d(1.0, 1.0, 1.0);
      assertFalse(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {0.0, 0.0, 1.0})));
      assertTrue(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {0.5, 0.0, 0.1})));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleMethodCalls()
   {
      Ramp3d ramp3d = new Ramp3d(1.0, 1.0, 1.0);

      // can apply transform and test a point that switches sides of the ramp
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(Math.PI / 2.0);

      Point3D p1 = new Point3D(new double[] {0.4, 0.2, 0.0});
      Point3D p2 = new Point3D(new double[] {0.4, -0.2, 0.0});

      assertTrue(ramp3d.isInsideOrOnSurface(p1));
      assertTrue(ramp3d.isInsideOrOnSurface(p2));

      ramp3d.applyTransform(transform);
      assertTrue(ramp3d.isInsideOrOnSurface(p1));
      assertFalse(ramp3d.isInsideOrOnSurface(p2));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsInsideOrOnSurface()
   {
      // With default epsilon
      Ramp3d ramp3d = new Ramp3d(new RigidBodyTransform(), 1.0, 1.0, 1.0);
      assertFalse(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {0.0, 0.0, 1.0})));
      assertFalse(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {0.0, 5.0, 1.0})));
      assertFalse(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {0.0, -5.0, 1.0})));
      assertTrue(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {0.5, 0.0, 0.2})));
      assertTrue(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {1.0, 0.3, 0.8})));

      // With finite epsilon
      assertTrue(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {0.0, 0.0, 1.0}), 1.0 / Math.sqrt(2.0) + 0.001));
      assertFalse(ramp3d.isInsideOrOnSurface(new Point3D(new double[] {0.0, 0.0, 1.0}), 1.0 / Math.sqrt(2.0) - 0.001));

      // With default epsilon and translation
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(0.75 * Math.PI);
      transform.setTranslation(new Vector3D(1.0, -1.0, 2.0));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testProjectionPerpNormal()
   {
      int iterations = 1000;

      Random random = new Random(8462L);

      // make sure the projection of a point is perpendicular to the surface, when it is above the surface.
      // normal
      for (int i = 0; i < iterations; i++)
      {
         Ramp3d ramp = createRandomRamp(random);
         Vector3D surfaceNormal = new Vector3D();
         ramp.getSurfaceNormal(surfaceNormal);
         
         double insideRamp = 0.1;
         double minX = insideRamp;
         double maxX = ramp.getRampLength() - insideRamp;
         double minY = -(ramp.getWidth() / 2.0) + insideRamp;
         double maxY = (ramp.getWidth() / 2.0) - insideRamp;
         double minZ = 0.0;
         double maxZ = 1.0;
         
         Point3D pointToTestOnRamp = RandomTools.generateRandomPoint(random, minX, minY, minZ, maxX, maxY, maxZ);
         pointToTestOnRamp = transformFromAngledToWorldFrame(ramp, pointToTestOnRamp);
         ramp.orthogonalProjection(pointToTestOnRamp);

         Point3D rampOrigin = new Point3D();
         rampOrigin = transformFromAngledToWorldFrame(ramp, rampOrigin);
        
         
         Vector3D vectorFromOriginToPointOnRamp = new Vector3D(pointToTestOnRamp);
         vectorFromOriginToPointOnRamp.sub(rampOrigin);
         
         double dotProduct = vectorFromOriginToPointOnRamp.dot(surfaceNormal);
         if (Math.abs(dotProduct) > 1e-14)
         {
            System.out.println(" ramp = " + ramp);
            System.out.println(" pointToTestOnRamp = " + pointToTestOnRamp);
            System.out.println(" rampOrigin = " + rampOrigin);
            System.out.println(" vectorFromOriginToPointOnRamp = " + vectorFromOriginToPointOnRamp);
            System.out.println(" surfaceNormal = " + surfaceNormal);
         }
         assertEquals(0.0, dotProduct, 1e-14);
      }
   }

	/**
    * Ramp3d needs a little more work and the tests improve. It's hard to do really good surface normal tests at the corners.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 502)
   public void testIsInsideOrOnSurfaceRandomOrientations()
   {
      int iterations = 1000;

      Random random = new Random(314159L);
      Ramp3d ramp = new Ramp3d(1.0, 1.0, 1.0);
      Point3D pointToTest = new Point3D();
      double rampLength;
      double epsilon;

      for (int i = 0; i < iterations; i++)
      {
         ramp = createRandomRamp(random);
         rampLength = Math.sqrt(MathTools.square(ramp.getHeight()) + MathTools.square(ramp.getLength()));
         epsilon = random.nextDouble();

         // z > 0 (in angled frame) means it's outside the ramp
         pointToTest.set(random.nextDouble(), random.nextDouble(), Math.abs(random.nextDouble()) + 2e-7);
         assertFalse(ramp.isInsideOrOnSurface(transformFromAngledToWorldFrame(ramp, pointToTest)));

         // negative x (in angled frame) means it's outside the ramp
         pointToTest.set(-Math.abs(random.nextDouble()) - 2e-7, random.nextDouble(), random.nextDouble());
         assertFalse(ramp.isInsideOrOnSurface(transformFromAngledToWorldFrame(ramp, pointToTest)));

         // x > ramp length (in angled frame) means it's outside the ramp
         pointToTest.set(Math.abs(random.nextDouble()) + rampLength + 2e-7, random.nextDouble(), random.nextDouble());
         assertFalse(ramp.isInsideOrOnSurface(transformFromAngledToWorldFrame(ramp, pointToTest)));

         // |y| > 0.5 * width (in both angled and ramp frames) means it's
         // outside the ramp
         pointToTest.set(random.nextDouble(), 0.5 * ramp.getWidth() + Math.abs(random.nextDouble()) + 2e-7, random.nextDouble());
         assertFalse(ramp.isInsideOrOnSurface(transformFromAngledToWorldFrame(ramp, pointToTest)));

         pointToTest.set(random.nextDouble(), -0.5 * ramp.getWidth() - Math.abs(random.nextDouble()) + 2e-7, random.nextDouble());
         assertFalse(ramp.isInsideOrOnSurface(transformFromAngledToWorldFrame(ramp, pointToTest)));

         // x < 0 (in ramp frame) means it's outside the ramp
         pointToTest.set(-random.nextDouble() - 2e-7, random.nextDouble(), random.nextDouble());
         ramp.getTransformUnsafe().transform(pointToTest);
         assertFalse(ramp.isInsideOrOnSurface(pointToTest));

         // x > length (in ramp frame) means it's outside the ramp
         pointToTest.set(Math.abs(random.nextDouble()) + ramp.getLength() + 2e-7, random.nextDouble(), random.nextDouble());
         ramp.getTransformUnsafe().transform(pointToTest);
         assertFalse(ramp.isInsideOrOnSurface(pointToTest));

         // z < 0 (in ramp frame) means it's outside the ramp
         pointToTest.set(random.nextDouble(), random.nextDouble(), -random.nextDouble() - 2e-7);
         ramp.getTransformUnsafe().transform(pointToTest);
         assertFalse(ramp.isInsideOrOnSurface(pointToTest));

         // z > height (in ramp frame) means it's outside the ramp
         pointToTest.set(random.nextDouble(), random.nextDouble(), Math.abs(random.nextDouble()) + ramp.getHeight() + 2e-7);
         ramp.getTransformUnsafe().transform(pointToTest);
         assertFalse(ramp.isInsideOrOnSurface(pointToTest));

         // points below the ramp surface (z < 0 in angled frame) are inside
         pointToTest.set(random.nextDouble() * rampLength, (random.nextDouble() - 0.5) * ramp.getWidth(), random.nextDouble() * epsilon);
         assertTrue(ramp.isInsideOrOnSurface(transformFromAngledToWorldFrame(ramp, pointToTest), epsilon));

         // points barely inside the side (y < 0.5*width in ramp frame and x and z so that they fit into the triangular sides)
         pointInsideRampSide(pointToTest, ramp, epsilon);
         assertTrue(ramp.isInsideOrOnSurface(pointToTest, epsilon));

         // points barely above the base (small positive z in ramp frame and x and z so that they fit into the rectangular sides) are inside
         pointToTest.set(random.nextDouble() * ramp.getLength(), 0.5 * random.nextDouble() * ramp.getWidth(), random.nextDouble() * epsilon);
         ramp.getTransformUnsafe().transform(pointToTest);
         assertTrue(ramp.isInsideOrOnSurface(pointToTest, epsilon));

         // points barely inside of the 'backboard', ie x = length
         pointToTest.set(random.nextDouble() * epsilon + ramp.getLength(), 0.5 * random.nextDouble() * ramp.getWidth(), random.nextDouble() * ramp.getHeight());
         ramp.getTransformUnsafe().transform(pointToTest);
         assertTrue(ramp.isInsideOrOnSurface(pointToTest, epsilon));
      }
   }

   /**
    * Ramp3d needs a little more work and the tests improve. It's hard to do really good surface normal tests at the corners.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 502)
	public void testTrickyOneThatProjectsOntoTheEdge()
	{
	  double width = 2.1722197228830327;
	  double length = 6.070053014293947;
	  double height = 0.6127921569306827;

	  RigidBodyTransform transform = new RigidBodyTransform(new double[]{
            0.999,  -0.026,   0.022, -0.260, 
            0.029,   0.990,  -0.137,  0.083, 
            -0.018,   0.138,   0.990, -0.029, 
            0,       0,       0,      1});
      
      Ramp3d ramp = new Ramp3d(transform, width, length, height);

      Point3D pointOutside = new Point3D(-0.836744392764021, 0.4813950690183002, -0.7222891854951903);
      Vector3D surfaceNormal = new Vector3D();
      Point3D closestPointToPack = new Point3D();
      ramp.checkIfInside(pointOutside, closestPointToPack, surfaceNormal);
      
      Point3D projectionOntoSurface = new Point3D(pointOutside);
      ramp.orthogonalProjection(projectionOntoSurface);

      EuclidCoreTestTools.assertTuple3DEquals(projectionOntoSurface, closestPointToPack, 1e-7);
      
      Point3D newClosestToPack = new Point3D();
      Vector3D newNormal = new Vector3D();
      ramp.checkIfInside(closestPointToPack, newClosestToPack, newNormal);
      
      EuclidCoreTestTools.assertTuple3DEquals(closestPointToPack, newClosestToPack, 1e-4);
      EuclidCoreTestTools.assertTuple3DEquals(surfaceNormal, newNormal, 1e-4);

      Point3D expectedClosestPoint = new Point3D(-0.2681570466884828, 0.393595239292229, 0.014295093961947069);
      Vector3D regressionNormal = new Vector3D(-0.9966461747460685, 0.07985119153963215, -0.01789384178230288);
      
      EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, closestPointToPack, 1e-4);
      EuclidCoreTestTools.assertTuple3DEquals(regressionNormal, newNormal, 1e-3);
	}
	
	
   /**
    * Ramp3d needs a little more work and the tests improve. It's hard to do really good surface normal tests at the corners.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 502)
   public void testDistance()
   {
      int iterations = 1000;

      Random random = new Random(26535L);

      for (int i = 0; i < iterations; i++)
      {
         Ramp3d ramp = createRandomRamp(random);
         
         double insideRamp = 0.02;
         Point3D pointToTestAboveRamp = RandomTools.generateRandomPoint(random, insideRamp, -ramp.getWidth()/2.0 + insideRamp, insideRamp, ramp.getRampLength() - insideRamp, ramp.getWidth()/2.0 + insideRamp, 1.0);
         Point3D pointOnRampBelowTestPoint = new Point3D(pointToTestAboveRamp);
         pointOnRampBelowTestPoint.setZ(0.0);
         double heightAboveRamp = pointToTestAboveRamp.getZ();
         
         pointToTestAboveRamp = transformFromAngledToWorldFrame(ramp, pointToTestAboveRamp);
         pointOnRampBelowTestPoint = transformFromAngledToWorldFrame(ramp, pointOnRampBelowTestPoint);

         double distanceUsingPoints = pointToTestAboveRamp.distance(pointOnRampBelowTestPoint);
         
         double distance = ramp.distance(pointToTestAboveRamp);
         

         
         if (Math.abs(distance - heightAboveRamp) > 1e-7)
         {
            System.out.println("distanceUsingPoints = " + distanceUsingPoints);
            System.out.println("distance = " + distance);
            System.out.println("heightAboveRamp = " + heightAboveRamp);
            
            distance = ramp.distance(pointToTestAboveRamp);

         }
         assertEquals(distance, heightAboveRamp, 1e-3);
      }
   }

   /**
    * Ramp3d needs a little more work and the tests improve. It's hard to do really good surface normal tests at the corners.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 502)
   public void testGetClosestPointAndNormalAt()
   {
      int iterations = 1000;

      Random random = new Random(897932L);
      Point3D pointToPack = new Point3D();
      Vector3D normalToPack = new Vector3D();

      // points that can be projected directly (ie just set z=0 in angled frame) give surface normal
      for (int i = 0; i < iterations; i++)
      {
         Ramp3d ramp = createRandomRamp(random);
         printIfDebug("\nramp = " + ramp);

         double insideRamp = 0.02;
         Point3D pointToTestAboveRamp = RandomTools.generateRandomPoint(random, insideRamp, -ramp.getWidth()/2.0 + insideRamp, insideRamp, ramp.getRampLength() - insideRamp, ramp.getWidth()/2.0 + insideRamp, 1.0);
         pointToTestAboveRamp = transformFromAngledToWorldFrame(ramp, pointToTestAboveRamp);

         printIfDebug("rampLength = " + ramp.getRampLength());
         printIfDebug("pointToTestAboveRamp = " + pointToTestAboveRamp);

         boolean isInside = ramp.checkIfInside(pointToTestAboveRamp, pointToPack, normalToPack);
         assertFalse(isInside);
         
         double distanceToPointToTest = ramp.distance(pointToTestAboveRamp);
         normalToPack.scale(distanceToPointToTest);
         
         Point3D pointOnRamp = new Point3D(pointToTestAboveRamp);
         pointOnRamp.sub(normalToPack);

         printIfDebug("pointOnRamp = " + pointOnRamp);
         double distanceToPointOnRamp = ramp.distance(pointOnRamp);
         printIfDebug("distanceToPointOnRamp = " + distanceToPointOnRamp);

         Point3D testPointOnRamp = new Point3D(pointToTestAboveRamp);
         ramp.orthogonalProjection(testPointOnRamp);
         
         Point3D testPointOnRampAgain = new Point3D(testPointOnRamp);
         ramp.orthogonalProjection(testPointOnRampAgain);
         printIfDebug("testPointOnRampAgain = " + testPointOnRampAgain);

         printIfDebug("testPointOnRamp = " + testPointOnRamp);
         double distanceToTestPointOnRamp = ramp.distance(testPointOnRamp);
         printIfDebug("distanceToTestPointOnRamp = " + distanceToTestPointOnRamp);

         EuclidCoreTestTools.assertTuple3DEquals(testPointOnRamp, pointOnRamp, 1e-7);
         
         printIfDebug("isInside = " + isInside);
         printIfDebug("distanceToPointToTest = " + distanceToPointToTest);

         assertEquals(distanceToPointOnRamp, 0.0, 1e-7);
      }
   }



	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIndependenceOfCopiedTransforms()
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI / 6);
      Ramp3d ramp = new Ramp3d(transform, 1.0, 1.0, 1.0);

      Ramp3d rampCopy = new Ramp3d(ramp);
      RigidBodyTransform transformAppliedOnlyToCopy = new RigidBodyTransform();
      transformAppliedOnlyToCopy.setRotationPitchAndZeroTranslation(Math.PI / 4);
      rampCopy.applyTransform(transformAppliedOnlyToCopy);
      assertFalse(rampCopy.getTransformUnsafe().equals(ramp.getTransformUnsafe()));

      Ramp3d rampCopyBySet = new Ramp3d(5.0, 6.0, 7.0);
      rampCopyBySet.set(ramp);
      RigidBodyTransform transformAppliedOnlyToCopyBySet = new RigidBodyTransform();
      transformAppliedOnlyToCopyBySet.setRotationYawAndZeroTranslation(Math.PI / 5);
      rampCopyBySet.applyTransform(transformAppliedOnlyToCopyBySet);
      assertFalse(rampCopyBySet.getTransformUnsafe().equals(ramp.getTransformUnsafe()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetMethodSetsUpAllFieldsOfNewRampAccurately()
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI / 6);
      Ramp3d ramp = new Ramp3d(transform, 2.0, 3.0, 4.0);
      Ramp3d rampCopyBySet = new Ramp3d(5.0, 6.0, 7.0);
      rampCopyBySet.set(ramp);
      Point3D pointProjectedOntoRamp = new Point3D(1.0, 0.0, 5.0);
      Point3D pointProjectedOntoRampCopy = new Point3D(pointProjectedOntoRamp);
      ramp.orthogonalProjection(pointProjectedOntoRamp);
      rampCopyBySet.orthogonalProjection(pointProjectedOntoRampCopy);
      assertEquals(pointProjectedOntoRamp, pointProjectedOntoRampCopy);
   }

   private static Ramp3d createRandomRamp(Random random)
   {
      RigidBodyTransform configuration = createRandomTransform(random);      
      
      double width = RandomTools.generateRandomDouble(random, 0.05, 1.0); 
      double length = RandomTools.generateRandomDouble(random, 0.05, 1.0); 
      double height = RandomTools.generateRandomDouble(random, 0.05, 1.0); 

      return new Ramp3d(configuration, length, width, height);
   }

   private static RigidBodyTransform createRandomTransform(Random random)
   {
      RigidBodyTransform transformReturn = new RigidBodyTransform();
      RigidBodyTransform transformTemp = new RigidBodyTransform();

      transformReturn.setRotationRollAndZeroTranslation(random.nextDouble());
      transformTemp.setRotationPitchAndZeroTranslation(random.nextDouble());
      transformReturn.multiply(transformTemp);
      transformTemp.setRotationYawAndZeroTranslation(random.nextDouble());
      transformReturn.multiply(transformTemp);
      transformReturn.setTranslation(new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));

      return transformReturn;
   }

   private static Point3D transformFromAngledToWorldFrame(Ramp3d ramp, Point3D point)
   {
      RigidBodyTransform angleTransform = new RigidBodyTransform();
      angleTransform.setRotationPitchAndZeroTranslation(-ramp.getRampIncline());
      angleTransform.transform(point);
      
      ramp.getTransformToWorldUnsafe().transform(point);
      
      return new Point3D(point);
   }

   private static void pointInsideRampSide(Point3D pointToPack, Ramp3d ramp, double epsilon)
   {
      Random random = new Random(97932L);

      double xVal = random.nextDouble() * ramp.getLength();
      double yVal = (2 * random.nextInt(1) - 1) * 0.5 * ramp.getWidth() + random.nextDouble() * epsilon;
      double zVal = random.nextDouble() * ramp.getHeight();

      if (zVal > xVal * (ramp.getHeight() / ramp.getLength()))
         zVal = xVal * (ramp.getHeight() / ramp.getLength());

      pointToPack.set(xVal, yVal, zVal);
      ramp.getTransformUnsafe().transform(pointToPack);
   }
   
   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
   }

}
