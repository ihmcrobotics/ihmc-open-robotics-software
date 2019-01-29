package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePoint3D;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameQuaternion;


public class ConstantPoseTrajectoryGeneratorTest
{
   private static final double EPSILON = 1e-12;

   private YoVariableRegistry registry = new YoVariableRegistry("registry");
   private ReferenceFrame referenceFrame;
   private FramePoint3DBasics positionMultipleFrames;
   private FrameQuaternionBasics orientationMultipleFrames;
   private ReferenceFrame rootFrame1 = ReferenceFrame.constructARootFrame("root1");
   private ConstantPoseTrajectoryGenerator generator;

   @Before
   public void setUp()
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      referenceFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("referenceFrame", rootFrame1, transformToParent);
      positionMultipleFrames = new YoMutableFramePoint3D("positionMultipleFrames", "", registry);
      orientationMultipleFrames = new YoMutableFrameQuaternion("orientationMultipleFrames", "", registry);
      generator = new ConstantPoseTrajectoryGenerator(positionMultipleFrames, orientationMultipleFrames);
      generator.switchTrajectoryFrame(referenceFrame);
   }

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
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
   public void testSetConstantPose()
   {

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsDone()
   {
      ConstantPoseTrajectoryGenerator generator1 = new ConstantPoseTrajectoryGenerator(positionMultipleFrames, orientationMultipleFrames);
      assertTrue(generator1.isDone());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGet()
   {
      FramePoint3D positionToPack = new FramePoint3D();
      generator.getPosition(positionToPack);
      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameQuaternion orientationToPack = new FrameQuaternion();
      generator.getOrientation(orientationToPack);
      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackVelocity()
   {
      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FramePoint3D positionToPack = new FramePoint3D(referenceFrame);
      positionToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      FrameQuaternion orientationToPack = new FrameQuaternion(referenceFrame);
      orientationToPack.setYawPitchRollIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.constructARootFrame("root"), 10.0, 10.0, 10.0);

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
      String expectedString = "Current position: " + positionMultipleFrames.toString() + "\nCurrent orientation: " + orientationMultipleFrames.toString();
      assertEquals(expectedString, generator.toString());
   }
}