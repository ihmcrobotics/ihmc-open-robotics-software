package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.ConstantOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ConstantOrientationTrajectoryGeneratorTest
{
   private static final double EPSILON = 1e-10;

   private ConstantOrientationTrajectoryGenerator generator;

   private String namePrefix = "namePrefixTEST";

   private ReferenceFrame referenceFrame;
   private OrientationProvider orientationProvider;
   private static double finalTime = 10.0;
   private YoVariableRegistry parentRegistry;

   private FrameOrientation orientation;

   @Before
   public void setUp()
   {
      referenceFrame = ReferenceFrame.constructARootFrame("rootNameTEST");
      orientation = new FrameOrientation(referenceFrame);
      orientationProvider = new ConstantOrientationProvider(orientation);
      parentRegistry = new YoVariableRegistry("parentRegistryTEST");
   }

   @After
   public void tearDown()
   {
      referenceFrame = null;
      orientation = null;
      orientationProvider = null;
      parentRegistry = null;
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testConstructor()
   {
      try
      {
         generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }

      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testIsDone()
   {
      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      generator.initialize();
      generator.compute(5.0);
      assertFalse(generator.isDone());

      generator.compute(finalTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testGet()
   {
      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      FrameOrientation orientationToPack = new FrameOrientation();

      generator.get(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testPackAngularVelocity()
   {
      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));

      generator.packAngularVelocity(angularVelocityToPack);

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testPackAngularAcceleration()
   {
      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));

      generator.packAngularAcceleration(angularAccelerationToPack);

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testPackAngularData()
   {
      FrameOrientation orientationToPack = new FrameOrientation(referenceFrame);
      orientationToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      generator.get(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.get(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(angularVelocityToPack.getReferenceFrame()));

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(angularAccelerationToPack.getReferenceFrame()));

      generator.packAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);

      assertEquals(0.0, orientationToPack.getYaw(), EPSILON);
      assertEquals(0.0, orientationToPack.getPitch(), EPSILON);
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
}