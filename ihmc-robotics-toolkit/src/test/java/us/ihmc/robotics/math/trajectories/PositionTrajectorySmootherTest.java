package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoRegistry;
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
   private FramePoint3D position;
   private YoRegistry parentRegistry;

   private double xValue = Math.random();
   private double yValue = Math.random();
   private double zValue = Math.random();

   private static double finalTime = 10.0;
   private static double dt = 0.001;

   @BeforeEach
   public void setUp()
   {
      parentRegistry = new YoRegistry("parentRegistryTEST");
      referenceFrame = ReferenceFrameTools.constructARootFrame("rootNameTEST");
      position = new FramePoint3D(referenceFrame, xValue, yValue, zValue);
      positionProvider = new ConstantPositionProvider(position);
      positionTrajectoryInput = new ConstantPositionTrajectoryGenerator(namePrefix, referenceFrame, positionProvider, finalTime, parentRegistry);
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
         smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }

      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
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
      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      smoother.initialize();
      smoother.compute(5.0);
      assertFalse(smoother.isDone());

      smoother.compute(finalTime + EPSILON);
      assertTrue(smoother.isDone());
   }

	@Test
   public void testGet()
   {
      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      FramePoint3D positionToPack = new FramePoint3D();

      smoother.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());
   }

	@Test
   public void testPackVelocity()
   {
      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(velocityToPack.getReferenceFrame()));

      smoother.getVelocity(velocityToPack);

      assertEquals(0.0, velocityToPack.getX(), EPSILON);
      assertEquals(0.0, velocityToPack.getY(), EPSILON);
      assertEquals(0.0, velocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, velocityToPack.getReferenceFrame());
   }

	@Test
   public void testPackAcceleration()
   {
      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));

      smoother.getAcceleration(accelerationToPack);

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

      smoother = new PositionTrajectorySmoother(namePrefix, positionTrajectoryInput, dt, parentRegistry);
      smoother.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      smoother.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

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
