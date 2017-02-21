package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameOrientationTest
{
   private RigidBodyTransform testTransform;
   private ReferenceFrame testFrame;
   private double epsilon = 1e-7;


   @Before
   public void setUp() throws Exception
   {
      testTransform = new RigidBodyTransform();
      testTransform.setTranslation(new Vector3D(0.0, 0.0, 1.0));
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
	      transformFromOneToTwoOriginal.setTranslation(new Vector3D());

	      RigidBodyTransform transformFromTwoToOneOriginal = frameTwo.getTransformToDesiredFrame(frameOne);
	      transformFromTwoToOneOriginal.setTranslation(new Vector3D());

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


		   EuclidCoreTestTools.assertRigidBodyTransformEquals(transformFromOneToTwoOriginal, transformFromOneToTwoExpected, 1e-7);
		   EuclidCoreTestTools.assertRigidBodyTransformEquals(transformFromOneToTwoOriginal, transformFromOneToTwo, 1e-7);

		   EuclidCoreTestTools.assertRigidBodyTransformEquals(transformFromTwoToOneOriginal, transformFromTwoToOneExpected, 1e-7);
		   EuclidCoreTestTools.assertRigidBodyTransformEquals(transformFromTwoToOneOriginal, transformFromTwoToOne, 1e-7);

	      EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(fromOneToTwo.getQuaternionCopy(), expectedFromOneToTwo.getQuaternionCopy(), 1e-3);
	      EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(fromTwoToOne.getQuaternionCopy(), expectedFromTwoToOne.getQuaternionCopy(), 1e-3);
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
         RigidBodyTransform originalTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         originalTransform.setTranslation(0.0, 0.0, 0.0);
         ReferenceFrame randomFrame_A = ReferenceFrame.generateRandomReferenceFrame("randomFrameA" + i, random, worldFrame);
         ReferenceFrame randomFrame_B = ReferenceFrame.generateRandomReferenceFrame("randomFrameB" + i, random, worldFrame);
         RigidBodyTransform randomTransformToWorld = randomFrame_B.getTransformToDesiredFrame(worldFrame);
         randomTransformToWorld.setTranslation(0.0, 0.0, 0.0);
         Quaternion randomQuaternionForTransformToWorld = new Quaternion();
         randomTransformToWorld.getRotation(randomQuaternionForTransformToWorld);

         testedFrameOrientation.setIncludingFrame(randomFrame_B, originalTransform);
         testedFrameOrientation.applyTransform(randomTransformToWorld);
         testedFrameOrientation.getTransform3D(actualTransformedTransform);

         expectedTransformedTransform.set(randomTransformToWorld);
         expectedTransformedTransform.multiply(originalTransform);
         
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransformedTransform, actualTransformedTransform, epsilon);

         RigidBodyTransform randomTransformToA = randomFrame_B.getTransformToDesiredFrame(randomFrame_A);
         randomTransformToA.setTranslation(0.0, 0.0, 0.0);
         Quaternion randomQuaternionForTransformToA = new Quaternion();
         randomTransformToA.getRotation(randomQuaternionForTransformToA);

         testedFrameOrientation.setIncludingFrame(randomFrame_B, originalTransform);
         testedFrameOrientation.applyTransform(randomTransformToA);
         testedFrameOrientation.getTransform3D(actualTransformedTransform);

         expectedTransformedTransform.set(randomTransformToA);
         expectedTransformedTransform.multiply(originalTransform);

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransformedTransform, actualTransformedTransform, epsilon);
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
         RigidBodyTransform originalTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         originalTransform.setTranslation(0.0, 0.0, 0.0);
         ReferenceFrame randomFrame_A = ReferenceFrame.generateRandomReferenceFrame("randomFrameA" + i, random, worldFrame);
         ReferenceFrame randomFrame_B = ReferenceFrame.generateRandomReferenceFrame("randomFrameB" + i, random, worldFrame);
         RigidBodyTransform randomTransformToWorld = randomFrame_B.getTransformToDesiredFrame(worldFrame);
         randomTransformToWorld.setTranslation(0.0, 0.0, 0.0);
         Quaternion randomQuaternionForTransformToWorld = new Quaternion();
         randomTransformToWorld.getRotation(randomQuaternionForTransformToWorld);

         testedFrameOrientation.setIncludingFrame(randomFrame_B, originalTransform);
         testedFrameOrientation.changeFrame(worldFrame);
         testedFrameOrientation.getTransform3D(actualTransformedTransform);

         expectedTransformedTransform.set(randomTransformToWorld);
         expectedTransformedTransform.multiply(originalTransform);
         
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransformedTransform, actualTransformedTransform, epsilon);

         RigidBodyTransform randomTransformToA = randomFrame_B.getTransformToDesiredFrame(randomFrame_A);
         randomTransformToA.setTranslation(0.0, 0.0, 0.0);
         Quaternion randomQuaternionForTransformToA = new Quaternion();
         randomTransformToA.getRotation(randomQuaternionForTransformToA);

         testedFrameOrientation.setIncludingFrame(randomFrame_B, originalTransform);
         testedFrameOrientation.changeFrame(randomFrame_A);
         testedFrameOrientation.getTransform3D(actualTransformedTransform);

         expectedTransformedTransform.set(randomTransformToA);
         expectedTransformedTransform.multiply(originalTransform);

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransformedTransform, actualTransformedTransform, epsilon);
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
		AxisAngle axisAngle = computeDifferenceAxisAngle(ori1, ori2);
		double angle = Math.abs(axisAngle.getAngle());
		assertTrue("difference angle: " + angle + " is greater than tolerance: " + angleTolerance, angle < angleTolerance);
	}

	public static AxisAngle computeDifferenceAxisAngle(FrameOrientation initialOrientation, FrameOrientation finalOrientation)
	{
		initialOrientation.checkReferenceFrameMatch(finalOrientation);
		Quaternion qDifference = initialOrientation.getQuaternionCopy();
		Quaternion q2 = finalOrientation.getQuaternionCopy();
		qDifference.inverse();
		qDifference.multiply(q2);
		AxisAngle axisAngle = new AxisAngle();
		axisAngle.set(qDifference);
		return axisAngle;
	}
}
