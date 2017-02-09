package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantPositionProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public class PositionTrajectorySmootherTest
{
   private static final double EPSILON = 1e-10;

   private String namePrefix = "namePrefixTEST";
   private ReferenceFrame referenceFrame;
   private PositionTrajectoryGenerator positionTrajectoryInput;
   private PositionTrajectorySmoother smoother;
   private PositionProvider positionProvider;
   private FramePoint position;
   private YoVariableRegistry parentRegistry;

   private double xValue = Math.random();
   private double yValue = Math.random();
   private double zValue = Math.random();

   private static double finalTime = 10.0;
   private static double dt = 0.001;

   @Before
   public void setUp()
   {
      parentRegistry = new YoVariableRegistry("parentRegistryTEST");
      referenceFrame = ReferenceFrame.constructARootFrame("rootNameTEST");
      position = new FramePoint(referenceFrame, xValue, yValue, zValue);
      positionProvider = new ConstantPositionProvider(position);
      positionTrajectoryInput = new ConstantPositionTrajectoryGenerator(namePrefix, referenceFrame, positionProvider, finalTime, parentRegistry);
   }

   @After
   public void tearDown()
   {

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      try
      {
         smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }

      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
   }

   //TODO: Implement this test
   //   @Test(timeout=300000)
   //   public void testComputeGains()
   //   {
   //      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
   //   }
   
//   @Test(timeout=300000)
//   public void testSetMaxAccelerationAndJerk()
//   {
//      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
//      double maxAbsoluteAcceleration = 10.0;
//      double maxAbsoluteJerk = 1.0;
//      
//      smoother.setMaxAccelerationAndJerk(maxAbsoluteAcceleration, maxAbsoluteJerk);
//   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsDone()
   {
      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      smoother.initialize();
      smoother.compute(5.0);
      assertFalse(smoother.isDone());

      smoother.compute(finalTime + EPSILON);
      assertTrue(smoother.isDone());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet()
   {
      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      FramePoint positionToPack = new FramePoint();

      smoother.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackVelocity()
   {
      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      FrameVector velocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));

      smoother.getVelocity(velocityToPack);

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, velocityToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAcceleration()
   {
      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));

      smoother.getAcceleration(accelerationToPack);

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

      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      smoother.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      smoother.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameVector velocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector accelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(velocityToPack.getReferenceFrame()));

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(accelerationToPack.getReferenceFrame()));

      smoother.getLinearData(positionToPack, velocityToPack, accelerationToPack);

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
}
