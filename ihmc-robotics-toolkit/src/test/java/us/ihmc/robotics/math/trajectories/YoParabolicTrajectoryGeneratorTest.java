package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.*;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.random.RandomGeometry;

public class YoParabolicTrajectoryGeneratorTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testConditions()
   {
      YoRegistry registry = new YoRegistry("registry");

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint3D initialPosition = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint3D finalPosition = new FramePoint3D(referenceFrame, 1.0, 1.0, 1.0);
      double intermediateParameter = 0.5;

      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
      trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);

      double delta = 1e-10;
      FramePoint3D positionToPack = new FramePoint3D(referenceFrame);

      trajectoryGenerator.getPosition(positionToPack, 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(initialPosition, positionToPack, delta);

      trajectoryGenerator.getPosition(positionToPack, intermediateParameter);
      EuclidCoreTestTools.assertTuple3DEquals(intermediatePosition, positionToPack, delta);

      trajectoryGenerator.getPosition(positionToPack, 1.0);
      EuclidCoreTestTools.assertTuple3DEquals(finalPosition, positionToPack, delta);
   }

	@Test
   public void testIllegalParameter1()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
      double intermediateParameter = 1.1;
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint3D initialPosition = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint3D finalPosition = new FramePoint3D(referenceFrame, 1.0, 1.0, 1.0);
      YoRegistry registry = new YoRegistry("registry");
      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
      trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);
      });
   }

	@Test
   public void testIllegalParameter2()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
      double intermediateParameter = -0.1;
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint3D initialPosition = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint3D finalPosition = new FramePoint3D(referenceFrame, 1.0, 1.0, 1.0);
      YoRegistry registry = new YoRegistry("registry");
      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
      trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);
      });
   }

	@Test
   public void testIllegalParameter3()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      YoParabolicTrajectoryGenerator trajectoryGenerator = null;

      try
      {
         double intermediateParameter = 0.7;
         FramePoint3D initialPosition = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
         FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, 0.5, 0.5, 2.5);
         FramePoint3D finalPosition = new FramePoint3D(referenceFrame, 1.0, 1.0, 1.0);
         YoRegistry registry = new YoRegistry("registry");
         trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
         trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);
      }
      catch (RuntimeException e)
      {
         fail();
      }

      FramePoint3D positionToPack = new FramePoint3D(referenceFrame);
      trajectoryGenerator.getPosition(positionToPack, 1.1);
      });
   }

	@Test
   public void testApex()
   {
      YoRegistry registry = new YoRegistry("registry");

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint3D initialPosition = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint3D finalPosition = new FramePoint3D(referenceFrame, 1.0, 1.0, 0.0);
      double intermediateParameter = 0.5;

      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
      trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);


      double delta = 1e-10;
      FramePoint3D positionToPack = new FramePoint3D(referenceFrame);
      int n = 1000;
      double smallestDifference = Double.POSITIVE_INFINITY;
      for (int i = 0; i < n; i++)
      {
         double parameter = i / (double) n;
         trajectoryGenerator.getPosition(positionToPack, parameter);
         double difference = intermediatePosition.getZ() - positionToPack.getZ();
         if (difference < smallestDifference)
            smallestDifference = difference;
      }

      assertTrue(smallestDifference < delta);
      assertTrue(smallestDifference >= 0.0);
   }

	@Test
   public void testVelocity()
   {
      YoRegistry registry = new YoRegistry("registry");
      Random random = new Random(186L);
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);

      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         FramePoint3D initialPosition = new FramePoint3D(referenceFrame, RandomGeometry.nextVector3D(random));
         FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, RandomGeometry.nextVector3D(random));
         FramePoint3D finalPosition = new FramePoint3D(referenceFrame, RandomGeometry.nextVector3D(random));
         double intermediateParameter = random.nextDouble();
         trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);

         FramePoint3D position1 = new FramePoint3D(referenceFrame);
         FramePoint3D position2 = new FramePoint3D(referenceFrame);

         double dt = 1e-9;
         double parameter = random.nextDouble();

         trajectoryGenerator.getPosition(position1, parameter);
         trajectoryGenerator.getPosition(position2, parameter + dt);

         FrameVector3D numericalVelocity = new FrameVector3D(position2);
         numericalVelocity.sub(position1);
         numericalVelocity.scale(1.0 / dt);

         FrameVector3D velocityFromTrajectoryGenerator = new FrameVector3D(referenceFrame);
         trajectoryGenerator.getVelocity(velocityFromTrajectoryGenerator, parameter);

         double delta = 1e-4;
         EuclidCoreTestTools.assertTuple3DEquals(numericalVelocity, velocityFromTrajectoryGenerator, delta);
      }
   }

	@Test
   public void testInitialVelocity()
   {
      YoRegistry registry = new YoRegistry("registry");
      Random random = new Random(186L);
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);

      FramePoint3D initialPosition = new FramePoint3D(referenceFrame, RandomGeometry.nextVector3D(random));
      FrameVector3D initialVelocity = new FrameVector3D(referenceFrame, RandomGeometry.nextVector3D(random));
      FramePoint3D finalPosition = new FramePoint3D(referenceFrame, RandomGeometry.nextVector3D(random));
      trajectoryGenerator.initialize(initialPosition, initialVelocity, finalPosition);

      FramePoint3D initialPositionBack = new FramePoint3D(referenceFrame);
      trajectoryGenerator.getPosition(initialPositionBack, 0.0);

      FrameVector3D initialVelocityBack = new FrameVector3D(referenceFrame);
      trajectoryGenerator.getVelocity(initialVelocityBack, 0.0);

      FramePoint3D finalPositionBack = new FramePoint3D(referenceFrame);
      trajectoryGenerator.getPosition(finalPositionBack, 1.0);

      double delta = 0.0;
      EuclidCoreTestTools.assertTuple3DEquals(initialPosition, initialPositionBack, delta);
      EuclidCoreTestTools.assertTuple3DEquals(initialVelocity, initialVelocityBack, delta);
      EuclidCoreTestTools.assertTuple3DEquals(finalPosition, finalPositionBack, delta);
   }
}
