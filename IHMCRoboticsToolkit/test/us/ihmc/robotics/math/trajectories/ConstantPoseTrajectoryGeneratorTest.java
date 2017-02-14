package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class ConstantPoseTrajectoryGeneratorTest
{
   private static final double EPSILON = 1e-12;

   private YoVariableRegistry registry = new YoVariableRegistry("registry");
   private final boolean allowMultipleFrames = true;
   private ReferenceFrame referenceFrame;
   private YoFramePoint positionYoFramePoint;
   private YoFrameQuaternion orientationQuaternion;
   private YoFramePointInMultipleFrames positionMultipleFrames;
   private YoFrameQuaternionInMultipleFrames orientationMultipleFrames;
   private ReferenceFrame rootFrame1 = ReferenceFrame.constructARootFrame("root1");
   private ConstantPoseTrajectoryGenerator generator;
   private ReferenceFrame frame2;

   @Before
   public void setUp()
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      //      referenceFrame = ReferenceFrame.constructARootFrame("rootFrame");
      referenceFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("referenceFrame", rootFrame1, transformToParent);

      positionYoFramePoint = new YoFramePoint("prefixTEST", referenceFrame, registry);
      orientationQuaternion = new YoFrameQuaternion("orientationPrefix", referenceFrame, registry);
      frame2 = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frame2", rootFrame1, transformToParent);
      positionMultipleFrames = new YoFramePointInMultipleFrames("positionMultipleFrames", registry, rootFrame1, frame2);
      orientationMultipleFrames = new YoFrameQuaternionInMultipleFrames("orientationMultipleFrames", registry, rootFrame1, frame2);
      generator = new ConstantPoseTrajectoryGenerator(positionYoFramePoint, orientationQuaternion);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructors()
   {
      ConstantPoseTrajectoryGenerator generator1 = new ConstantPoseTrajectoryGenerator(positionYoFramePoint, orientationQuaternion);
      ConstantPoseTrajectoryGenerator generator2 = new ConstantPoseTrajectoryGenerator(positionMultipleFrames, orientationMultipleFrames);
      ConstantPoseTrajectoryGenerator generator3 = new ConstantPoseTrajectoryGenerator("generator3", referenceFrame, registry);
      ConstantPoseTrajectoryGenerator generator4 = new ConstantPoseTrajectoryGenerator("generator4", allowMultipleFrames, referenceFrame, registry);

      try
      {
         generator1.registerNewTrajectoryFrame(referenceFrame);
         fail();
      }
      catch (RuntimeException rte)
      {
      }

      generator2.registerAndSwitchFrame(referenceFrame);

      try
      {
         generator3.registerNewTrajectoryFrame(referenceFrame);
         fail();
      }
      catch (RuntimeException rte)
      {
      }

      generator4.registerNewTrajectoryFrame(referenceFrame);

      try
      {
         ConstantPoseTrajectoryGenerator generator04 = new ConstantPoseTrajectoryGenerator("generator4", false, referenceFrame, registry);
         generator04.registerNewTrajectoryFrame(referenceFrame);
         fail();
      }
      catch (RuntimeException rte)
      {
      }

      try
      {
         YoFrameQuaternion orientationQuaternion2 = new YoFrameQuaternion("orientationPrefix2", ReferenceFrame.constructAWorldFrame("worldFrame"), registry);
         generator1 = null;
         generator1 = new ConstantPoseTrajectoryGenerator(positionYoFramePoint, orientationQuaternion2);
         fail();
      }
      catch (ReferenceFrameMismatchException rfme)
      {
      }

      try
      {
         YoFrameQuaternionInMultipleFrames orientationMultipleFrames2 = new YoFrameQuaternionInMultipleFrames("orientationMultipleFrames2", registry,
               ReferenceFrame.constructAWorldFrame("worldFrame2"), frame2);
         generator2 = null;
         generator2 = new ConstantPoseTrajectoryGenerator(positionMultipleFrames, orientationMultipleFrames2);
         fail();
      }
      catch (ReferenceFrameMismatchException rfme)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   //TODO: Find a way to test this.
   public void testRegisterNewTrajectoryFrame()
   {
      ConstantPoseTrajectoryGenerator generator2 = new ConstantPoseTrajectoryGenerator(positionMultipleFrames, orientationMultipleFrames);
      generator2.registerNewTrajectoryFrame(rootFrame1);

      //      System.out.println(generator2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   //TODO: Find a way to test this.
   public void testChangeFrame()
   {

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   //TODO: Find a way to test this.
   public void testSwitchTrajectoryFrame()
   {

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   //TODO: Find a way to test this.
   public void testRegisterAndSwitchFrame()
   {

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   //TODO: Find a way to test this.
   public void testSetConstantPose()
   {

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsDone()
   {
      ConstantPoseTrajectoryGenerator generator1 = new ConstantPoseTrajectoryGenerator(positionYoFramePoint, orientationQuaternion);
      assertTrue(generator1.isDone());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGet()
   {
      FramePoint positionToPack = new FramePoint();
      generator.getPosition(positionToPack);
      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameOrientation orientationToPack = new FrameOrientation();
      generator.getOrientation(orientationToPack);
      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackVelocity()
   {
      FrameVector velocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));

      generator.getVelocity(velocityToPack);

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, velocityToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackAcceleration()
   {
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));

      generator.getAcceleration(accelerationToPack);

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, accelerationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackAngularVelocity()
   {
      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));

      generator.getAngularVelocity(angularVelocityToPack);

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackAngularAcceleration()
   {
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));

      generator.getAngularAcceleration(angularAccelerationToPack);

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackLinearData()
   {
      FramePoint positionToPack = new FramePoint(referenceFrame);
      positionToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameVector velocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(velocityToPack.getReferenceFrame()));

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(accelerationToPack.getReferenceFrame()));

      generator.getLinearData(positionToPack, velocityToPack, accelerationToPack);

      assertEquals(0.0, positionToPack.getX(), EPSILON);
      assertEquals(0.0, positionToPack.getY(), EPSILON);
      assertEquals(0.0, positionToPack.getZ(), EPSILON);
      assertSame(referenceFrame, positionToPack.getReferenceFrame());

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, velocityToPack.getReferenceFrame());

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, accelerationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackAngularData()
   {
      FrameOrientation orientationToPack = new FrameOrientation(referenceFrame);
      orientationToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(angularVelocityToPack.getReferenceFrame()));

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(angularAccelerationToPack.getReferenceFrame()));

      generator.getAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);

      assertEquals(0.0, orientationToPack.getPitch(), EPSILON);
      assertEquals(0.0, orientationToPack.getYaw(), EPSILON);
      assertEquals(0.0, orientationToPack.getRoll(), EPSILON);
      assertSame(referenceFrame, orientationToPack.getReferenceFrame());

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testInitialize()
   {
      generator.initialize();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCompute()
   {
      generator.compute(0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      String expectedString = "Current position: " + positionYoFramePoint.toString() + "\nCurrent orientation: " + orientationQuaternion.toString();
      assertEquals(expectedString, generator.toString());
   }
}