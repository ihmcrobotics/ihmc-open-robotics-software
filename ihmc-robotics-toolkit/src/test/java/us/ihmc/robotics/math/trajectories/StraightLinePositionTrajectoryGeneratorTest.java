package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.robotics.trajectories.providers.FramePositionProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StraightLinePositionTrajectoryGeneratorTest
{

   private static final double EPSILON = 1e-10;

   private String namePrefix = "namePrefixTEST";
   private ReferenceFrame referenceFrame;
   private DoubleProvider trajectoryTimeProvider;
   private StraightLinePositionTrajectoryGenerator generator;
   private FramePositionProvider initialPositionProvider;
   private FramePositionProvider finalPositionProvider;
   private FramePoint3D position;
   private YoRegistry parentRegistry;

   private double xValue = Math.random();
   private double yValue = Math.random();
   private double zValue = Math.random();

   private static double finalTime = 10.0;
   @BeforeEach
   public void setUp()
   {
      parentRegistry = new YoRegistry("parentRegistryTEST");
      referenceFrame = ReferenceFrameTools.constructARootFrame("rootNameTEST");
      position = new FramePoint3D(referenceFrame, xValue, yValue, zValue);
      initialPositionProvider = () -> position;
      finalPositionProvider = () -> position;
      trajectoryTimeProvider = () -> 10.0;
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testConstructor()
   {
      try
      {
         generator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }

      generator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
   }

   //TODO: Implement this test
   //   @Test
   //   public void testComputeGains()
   //   {
   //      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
   //   }
   
//   @Test
//   public void testSetMaxAccelerationAndJerk()
//   {
//      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
//      double maxAbsoluteAcceleration = 10.0;
//      double maxAbsoluteJerk = 1.0;
//      
//      smoother.setMaxAccelerationAndJerk(maxAbsoluteAcceleration, maxAbsoluteJerk);
//   }

	@Test
   public void testIsDone()
   {
      generator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
      generator.initialize();
      generator.compute(5.0);
      assertFalse(generator.isDone());

      generator.compute(finalTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@Test
   public void testGet()
   {
      generator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
      FramePoint3D positionToPack = new FramePoint3D();

      positionToPack.setIncludingFrame(generator.getPosition());

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());
   }

	@Test
   public void testPackVelocity()
   {
      generator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));

      velocityToPack.setIncludingFrame(generator.getVelocity());

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, velocityToPack.getReferenceFrame());
   }

	@Test
   public void testPackAcceleration()
   {
      generator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));

      accelerationToPack.setIncludingFrame(generator.getAcceleration());

      assertEquals(0.0, accelerationToPack.getX(), EPSILON);
      assertEquals(0.0, accelerationToPack.getY(), EPSILON);
      assertEquals(0.0, accelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, accelerationToPack.getReferenceFrame());
   }

	@Test
   public void testPackLinearData()
   {
      FramePoint3D positionToPack = new FramePoint3D(referenceFrame);
      positionToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
      positionToPack.setIncludingFrame(generator.getPosition());

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      positionToPack.setIncludingFrame(generator.getPosition());

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

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
}
