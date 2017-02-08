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
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public class WrapperForPositionAndOrientationTrajectoryGeneratorsTest
{
   private static final double EPSILON = 1e-10;

   private ReferenceFrame referenceFrame;
   private WrapperForPositionAndOrientationTrajectoryGenerators generator;
   private PositionTrajectoryGenerator positionTrajectoryGenerator;
   private OrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private FrameOrientation orientation;
   private YoVariableRegistry parentRegistry;
   private OrientationProvider orientationProvider;
   private static double finalTime = 10.0;

   @Before
   public void setUp()
   {
      parentRegistry = new YoVariableRegistry("parentRegistryTEST");
      referenceFrame = ReferenceFrame.constructARootFrame("rootNameTEST");

      orientation = new FrameOrientation(referenceFrame);
      positionTrajectoryGenerator = new ConstantPoseTrajectoryGenerator("positionTGenPrefix", referenceFrame, parentRegistry);
      orientationProvider = new ConstantOrientationProvider(orientation);
      orientationTrajectoryGenerator = new ConstantOrientationTrajectoryGenerator("orientationPrefix", referenceFrame, orientationProvider, finalTime,
            parentRegistry);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInitialize()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(null, null);
      try
      {
         generator.initialize();
         fail("Should throw an exception");
      }
      catch (NullPointerException npe)
      {
      }

      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      generator.initialize();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCompute()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(null, null);
      try
      {
         generator.compute(10.0);
         fail("Should throw an exception");
      }
      catch (NullPointerException npe)
      {
      }

      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      generator.compute(10.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsDone()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      generator.initialize();
      generator.compute(5.0);
      assertFalse(generator.isDone());

      generator.compute(finalTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FramePoint positionToPack = new FramePoint();

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet_Orientation()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameOrientation orientationToPack = new FrameOrientation();

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackVelocity()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameVector velocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));

      generator.getVelocity(velocityToPack);

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, velocityToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAcceleration()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));

      generator.getAcceleration(accelerationToPack);

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, accelerationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackLinearData()
   {
      FramePoint positionToPack = new FramePoint(referenceFrame);
      positionToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameVector velocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(velocityToPack.getReferenceFrame()));

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(accelerationToPack.getReferenceFrame()));

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
	@Test(timeout = 30000)
   public void testPackAngularVelocity()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));

      generator.getAngularVelocity(angularVelocityToPack);

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAngularAcceleration()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));

      generator.getAngularAcceleration(angularAccelerationToPack);

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAngularData()
   {
      FrameOrientation orientationToPack = new FrameOrientation(referenceFrame);
      orientationToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector velocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(velocityToPack.getReferenceFrame()));

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(accelerationToPack.getReferenceFrame()));

      generator.getAngularData(orientationToPack, velocityToPack, accelerationToPack);

      assertEquals(0.0, orientationToPack.getYaw(), EPSILON);
      assertEquals(0.0, orientationToPack.getPitch(), EPSILON);
      assertEquals(0.0, orientationToPack.getRoll(), EPSILON);
      assertSame(referenceFrame, orientationToPack.getReferenceFrame());

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, velocityToPack.getReferenceFrame());

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, accelerationToPack.getReferenceFrame());
   }
}
