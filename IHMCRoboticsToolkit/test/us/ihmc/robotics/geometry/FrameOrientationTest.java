package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class FrameOrientationTest
{
   private RigidBodyTransform testTransform;
   private ReferenceFrame testFrame;
   private double epsilon = 1e-7;


   @Before
   public void setUp() throws Exception
   {
      testTransform = new RigidBodyTransform();
      testTransform.setTranslation(new Vector3d(0.0, 0.0, 1.0));
      testTransform.setRotationRollAndZeroTranslation(0.2);
      testTransform.setRotationPitchAndZeroTranslation(0.6);
      testTransform.setRotationYawAndZeroTranslation(0.8);
      testFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("testFrame", ReferenceFrame.getWorldFrame(), new RigidBodyTransform(), false, false,
              false);
   }

   @After
   public void tearDown() throws Exception
   {
      testTransform = null;
      testFrame = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOrientationOrientation()
   {
      FrameOrientation original = new FrameOrientation(testFrame);
      original.applyTransform(testTransform);
      original.setYawPitchRoll(0.1, 0.2, 0.3);

      FrameOrientation test = new FrameOrientation(original);

      RigidBodyTransform originalT = new RigidBodyTransform();
      original.getTransform3D(originalT);
      RigidBodyTransform testT = new RigidBodyTransform();
      test.getTransform3D(testT);

      assertEquals(originalT, testT);
      assertEquals(original.getYawPitchRoll()[0], test.getYawPitchRoll()[0], epsilon);
      assertEquals(original.getYawPitchRoll()[1], test.getYawPitchRoll()[1], epsilon);
      assertEquals(original.getYawPitchRoll()[2], test.getYawPitchRoll()[2], epsilon);
      assertEquals(original.getReferenceFrame(), testFrame);
      assertEquals(original.getReferenceFrame(), test.getReferenceFrame());
   }
	
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void testSetOrientationFromOneToTwo()
	{
	   Random random = new Random(1776L);

	   ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

	   int numberOfTests = 100;

	   for (int i=0; i<numberOfTests; i++)
	   {
	      ReferenceFrame frameOne = ReferenceFrame.generateRandomReferenceFrame("frameOne", random, worldFrame);
	      ReferenceFrame frameTwo = ReferenceFrame.generateRandomReferenceFrame("frameTwo", random, worldFrame);

	      RigidBodyTransform transformFromOneToTwoOriginal = frameOne.getTransformToDesiredFrame(frameTwo);
	      transformFromOneToTwoOriginal.setTranslation(new Vector3d());

	      RigidBodyTransform transformFromTwoToOneOriginal = frameTwo.getTransformToDesiredFrame(frameOne);
	      transformFromTwoToOneOriginal.setTranslation(new Vector3d());

	      FrameOrientation orientationOne = new FrameOrientation(frameOne);
	      FrameOrientation orientationTwo = new FrameOrientation(frameTwo);

	      orientationOne.changeFrame(worldFrame);
	      orientationTwo.changeFrame(worldFrame);

	      FrameOrientation fromOneToTwo = new FrameOrientation(worldFrame);
	      FrameOrientation fromTwoToOne = new FrameOrientation(worldFrame);

	      fromOneToTwo.setOrientationFromOneToTwo(orientationOne, orientationTwo);
	      fromTwoToOne.setOrientationFromOneToTwo(orientationTwo, orientationOne);

	      FrameOrientation expectedFromOneToTwo = new FrameOrientation(frameOne);
	      expectedFromOneToTwo.changeFrame(frameTwo);

	      FrameOrientation expectedFromTwoToOne = new FrameOrientation(frameTwo);
	      expectedFromTwoToOne.changeFrame(frameOne);

	      RigidBodyTransform transformFromOneToTwo = new RigidBodyTransform();
	      RigidBodyTransform transformFromOneToTwoExpected = new RigidBodyTransform();
	      fromOneToTwo.getTransform3D(transformFromOneToTwo);
	      expectedFromOneToTwo.getTransform3D(transformFromOneToTwoExpected);

	      RigidBodyTransform transformFromTwoToOne = new RigidBodyTransform();
	      RigidBodyTransform transformFromTwoToOneExpected = new RigidBodyTransform();
	      fromTwoToOne.getTransform3D(transformFromTwoToOne);
	      expectedFromTwoToOne.getTransform3D(transformFromTwoToOneExpected);


		   RigidBodyTransformTest.assertTransformEquals(transformFromOneToTwoOriginal, transformFromOneToTwoExpected, 1e-7);
		   RigidBodyTransformTest.assertTransformEquals(transformFromOneToTwoOriginal, transformFromOneToTwo, 1e-7);

		   RigidBodyTransformTest.assertTransformEquals(transformFromTwoToOneOriginal, transformFromTwoToOneExpected, 1e-7);
		   RigidBodyTransformTest.assertTransformEquals(transformFromTwoToOneOriginal, transformFromTwoToOne, 1e-7);

	      JUnitTools.assertQuaternionsEqualUsingDifference(fromOneToTwo.getQuaternionCopy(), expectedFromOneToTwo.getQuaternionCopy(), 1e-3);
	      JUnitTools.assertQuaternionsEqualUsingDifference(fromTwoToOne.getQuaternionCopy(), expectedFromTwoToOne.getQuaternionCopy(), 1e-3);
	   }
	}

//   @Test(timeout=300000)
//   public void testOrientationReferenceFrame()
//   {
//   }
//
//   @Test(timeout=300000)
//   public void testOrientationReferenceFrameTransform3D()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testOrientationReferenceFrameQuat4d()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testOrientationReferenceFrameQuat4f()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testOrientationReferenceFrameDoubleDoubleDoubleDouble()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testOrientationReferenceFrameDoubleDoubleDouble()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testOrientationReferenceFrameDoubleArray()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testCheckReferenceFrameMatchReferenceFrameHolder()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testCheckReferenceFrameMatchReferenceFrame()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testGetReferenceFrame()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testGetTransform3DCopy()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testSetQuat4f()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testSetQuat4d()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testSetYawPitchRollDoubleDoubleDouble()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testSetYawPitchRollDoubleArray()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testGetYawPitchRollDoubleArray()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testGetYawPitchRoll()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testGetYawPitchRollQuat4fDoubleArray()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testAverageOrientations()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testGetQuaternionQuat4d()
//   {
//      fail("Not yet implemented");    // TODO
//   }
//
//   @Test(timeout=300000)
//   public void testGetQuaternion()
//   {
//      fail("Not yet implemented");    // TODO
//   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout=1000)
   public void testApplyTransform()
   {
      Random random = new Random(56165161L);

      RigidBodyTransform expectedTransformedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransformedTransform = new RigidBodyTransform();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameOrientation testedFrameOrientation = new FrameOrientation();

      for (int i = 0; i < 100000; i ++)
      {
         RigidBodyTransform originalTransform = RigidBodyTransform.generateRandomTransform(random);
         originalTransform.setTranslation(0.0, 0.0, 0.0);
         ReferenceFrame randomFrame_A = ReferenceFrame.generateRandomReferenceFrame("randomFrameA" + i, random, worldFrame);
         ReferenceFrame randomFrame_B = ReferenceFrame.generateRandomReferenceFrame("randomFrameB" + i, random, worldFrame);
         RigidBodyTransform randomTransformToWorld = randomFrame_B.getTransformToDesiredFrame(worldFrame);
         randomTransformToWorld.setTranslation(0.0, 0.0, 0.0);
         Quat4d randomQuaternionForTransformToWorld = new Quat4d();
         randomTransformToWorld.getRotation(randomQuaternionForTransformToWorld);

         testedFrameOrientation.setIncludingFrame(randomFrame_B, originalTransform);
         testedFrameOrientation.applyTransform(randomTransformToWorld);
         testedFrameOrientation.getTransform3D(actualTransformedTransform);

         expectedTransformedTransform.multiply(randomTransformToWorld, originalTransform);
         
         RigidBodyTransformTest.assertTransformEquals(expectedTransformedTransform, actualTransformedTransform, epsilon);

         RigidBodyTransform randomTransformToA = randomFrame_B.getTransformToDesiredFrame(randomFrame_A);
         randomTransformToA.setTranslation(0.0, 0.0, 0.0);
         Quat4d randomQuaternionForTransformToA = new Quat4d();
         randomTransformToA.getRotation(randomQuaternionForTransformToA);

         testedFrameOrientation.setIncludingFrame(randomFrame_B, originalTransform);
         testedFrameOrientation.applyTransform(randomTransformToA);
         testedFrameOrientation.getTransform3D(actualTransformedTransform);

         expectedTransformedTransform.multiply(randomTransformToA, originalTransform);

         RigidBodyTransformTest.assertTransformEquals(expectedTransformedTransform, actualTransformedTransform, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout=1000)
   public void testChangeFrame()
   {
      Random random = new Random(56165161L);

      RigidBodyTransform expectedTransformedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransformedTransform = new RigidBodyTransform();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameOrientation testedFrameOrientation = new FrameOrientation();

      for (int i = 0; i < 100000; i ++)
      {
         RigidBodyTransform originalTransform = RigidBodyTransform.generateRandomTransform(random);
         originalTransform.setTranslation(0.0, 0.0, 0.0);
         ReferenceFrame randomFrame_A = ReferenceFrame.generateRandomReferenceFrame("randomFrameA" + i, random, worldFrame);
         ReferenceFrame randomFrame_B = ReferenceFrame.generateRandomReferenceFrame("randomFrameB" + i, random, worldFrame);
         RigidBodyTransform randomTransformToWorld = randomFrame_B.getTransformToDesiredFrame(worldFrame);
         randomTransformToWorld.setTranslation(0.0, 0.0, 0.0);
         Quat4d randomQuaternionForTransformToWorld = new Quat4d();
         randomTransformToWorld.getRotation(randomQuaternionForTransformToWorld);

         testedFrameOrientation.setIncludingFrame(randomFrame_B, originalTransform);
         testedFrameOrientation.changeFrame(worldFrame);
         testedFrameOrientation.getTransform3D(actualTransformedTransform);

         expectedTransformedTransform.multiply(randomTransformToWorld, originalTransform);
         
         RigidBodyTransformTest.assertTransformEquals(expectedTransformedTransform, actualTransformedTransform, epsilon);

         RigidBodyTransform randomTransformToA = randomFrame_B.getTransformToDesiredFrame(randomFrame_A);
         randomTransformToA.setTranslation(0.0, 0.0, 0.0);
         Quat4d randomQuaternionForTransformToA = new Quat4d();
         randomTransformToA.getRotation(randomQuaternionForTransformToA);

         testedFrameOrientation.setIncludingFrame(randomFrame_B, originalTransform);
         testedFrameOrientation.changeFrame(randomFrame_A);
         testedFrameOrientation.getTransform3D(actualTransformedTransform);

         expectedTransformedTransform.multiply(randomTransformToA, originalTransform);

         RigidBodyTransformTest.assertTransformEquals(expectedTransformedTransform, actualTransformedTransform, epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChangeFrameCopy()
   {
      FrameOrientation origOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());

      FrameOrientation newOrientation = new FrameOrientation(testFrame);

      origOrientation = new FrameOrientation(origOrientation);
      origOrientation.changeFrame(testFrame);

      assertEquals(origOrientation.getReferenceFrame(), newOrientation.getReferenceFrame());

   }

	public static void assertFrameOrientationEquals(FrameOrientation ori1, FrameOrientation ori2, double angleTolerance)
	{
		AxisAngle4d axisAngle = computeDifferenceAxisAngle(ori1, ori2);
		double angle = Math.abs(axisAngle.getAngle());
		assertTrue("difference angle: " + angle + " is greater than tolerance: " + angleTolerance, angle < angleTolerance);
	}

	public static AxisAngle4d computeDifferenceAxisAngle(FrameOrientation initialOrientation, FrameOrientation finalOrientation)
	{
		initialOrientation.checkReferenceFrameMatch(finalOrientation);
		Quat4d qDifference = initialOrientation.getQuaternionCopy();
		Quat4d q2 = finalOrientation.getQuaternionCopy();
		qDifference.inverse();
		qDifference.mul(q2);
		AxisAngle4d axisAngle = new AxisAngle4d();
		axisAngle.set(qDifference);
		return axisAngle;
	}
}
