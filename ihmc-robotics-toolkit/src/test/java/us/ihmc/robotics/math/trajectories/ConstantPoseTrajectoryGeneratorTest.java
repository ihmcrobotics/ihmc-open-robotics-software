package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertSame;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;


public class ConstantPoseTrajectoryGeneratorTest
{
   private static final double EPSILON = 1e-12;

   private YoRegistry registry = new YoRegistry("registry");
   private ReferenceFrame referenceFrame;
   private FramePoint3DBasics positionMultipleFrames;
   private FrameQuaternionBasics orientationMultipleFrames;
   private ReferenceFrame rootFrame1 = ReferenceFrameTools.constructARootFrame("root1");
   private ConstantPoseTrajectoryGenerator generator;

   @BeforeEach
   public void setUp()
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      referenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("referenceFrame", rootFrame1, transformToParent);
      positionMultipleFrames = new YoMutableFramePoint3D("positionMultipleFrames", "", registry);
      orientationMultipleFrames = new YoMutableFrameQuaternion("orientationMultipleFrames", "", registry);
      generator = new ConstantPoseTrajectoryGenerator(positionMultipleFrames, orientationMultipleFrames);
      generator.switchTrajectoryFrame(referenceFrame);
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   //TODO: Find a way to test this.
   public void testChangeFrame()
   {

   }

	@Test
   //TODO: Find a way to test this.
   public void testSwitchTrajectoryFrame()
   {

   }

	@Test
   //TODO: Find a way to test this.
   public void testSetConstantPose()
   {

   }

	@Test
   public void testIsDone()
   {
      ConstantPoseTrajectoryGenerator generator1 = new ConstantPoseTrajectoryGenerator(positionMultipleFrames, orientationMultipleFrames);
      assertTrue(generator1.isDone());
   }

	@Test
   public void testGet()
   {
      FramePoint3D positionToPack = new FramePoint3D();
      generator.getPosition(positionToPack);
      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameQuaternion orientationToPack = new FrameQuaternion();
      generator.getOrientation(orientationToPack);
      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@Test
   public void testPackVelocity()
   {
      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrameTools.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));

      generator.getVelocity(velocityToPack);

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, velocityToPack.getReferenceFrame());
   }

	@Test
   public void testPackAcceleration()
   {
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrameTools.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));

      generator.getAcceleration(accelerationToPack);

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, accelerationToPack.getReferenceFrame());
   }

	@Test
   public void testPackAngularVelocity()
   {
      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrameTools.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));

      generator.getAngularVelocity(angularVelocityToPack);

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

	@Test
   public void testPackAngularAcceleration()
   {
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrameTools.constructARootFrame("root"), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));

      generator.getAngularAcceleration(angularAccelerationToPack);

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@Test
   public void testPackLinearData()
   {
      FramePoint3D positionToPack = new FramePoint3D(referenceFrame);
      positionToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrameTools.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrameTools.constructARootFrame("root"), 10.0, 10.0, 10.0);

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

	@Test
   public void testPackAngularData()
   {
      FrameQuaternion orientationToPack = new FrameQuaternion(referenceFrame);
      orientationToPack.setYawPitchRollIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrameTools.constructARootFrame("root"), 10.0, 10.0, 10.0);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrameTools.constructARootFrame("root"), 10.0, 10.0, 10.0);

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

	@Test
   public void testInitialize()
   {
      generator.initialize();
   }

	@Test
   public void testCompute()
   {
      generator.compute(0.0);
   }

	@Test
   public void testToString()
   {
      String expectedString = "Current position: " + positionMultipleFrames.toString() + "\nCurrent orientation: " + orientationMultipleFrames.toString();
      assertEquals(expectedString, generator.toString());
   }
}