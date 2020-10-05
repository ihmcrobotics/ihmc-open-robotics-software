package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public class WrapperForPositionAndOrientationTrajectoryGeneratorsTest
{
   private static final double EPSILON = 1e-10;

   private ReferenceFrame referenceFrame;
   private WrapperForPositionAndOrientationTrajectoryGenerators generator;
   private PositionTrajectoryGenerator positionTrajectoryGenerator;
   private OrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private FrameQuaternion orientation;
   private YoRegistry parentRegistry;
   private OrientationProvider orientationProvider;
   private static double finalTime = 10.0;

   @BeforeEach
   public void setUp()
   {
      parentRegistry = new YoRegistry("parentRegistryTEST");
      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame(new Random(302L));

      orientation = new FrameQuaternion(referenceFrame);
      positionTrajectoryGenerator = new ConstantPoseTrajectoryGenerator("positionTGenPrefix", referenceFrame, parentRegistry);
      orientationProvider = new ConstantOrientationProvider(orientation);
      orientationTrajectoryGenerator = new ConstantOrientationTrajectoryGenerator("orientationPrefix", referenceFrame, orientationProvider, finalTime,
            parentRegistry);
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
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

	@Test
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

	@Test
   public void testIsDone()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      generator.initialize();
      generator.compute(5.0);
      assertFalse(generator.isDone());

      generator.compute(finalTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@Test
   public void testGet()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FramePoint3D positionToPack = new FramePoint3D();

      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());
   }

	@Test
   public void testGet_Orientation()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameQuaternion orientationToPack = new FrameQuaternion();

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@Test
   public void testPackVelocity()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

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
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(accelerationToPack.getReferenceFrame()));

      generator.getAcceleration(accelerationToPack);

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

      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      generator.getPosition(positionToPack);

      assertEquals(referenceFrame, positionToPack.getReferenceFrame());

      generator.getPosition(positionToPack);

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

	@Test
   public void testPackAngularVelocity()
   {
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

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
      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));

      generator.getAngularAcceleration(angularAccelerationToPack);

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@Test
   public void testPackAngularData()
   {
      FrameQuaternion orientationToPack = new FrameQuaternion(referenceFrame);
      orientationToPack.setYawPitchRollIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector3D velocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector3D accelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

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
