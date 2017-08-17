package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomGeometry;

public class YoParabolicTrajectoryGeneratorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConditions()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testIllegalParameter1()
   {
      double intermediateParameter = 1.1;
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint3D initialPosition = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint3D finalPosition = new FramePoint3D(referenceFrame, 1.0, 1.0, 1.0);
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
      trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testIllegalParameter2()
   {
      double intermediateParameter = -0.1;
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint3D initialPosition = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint3D finalPosition = new FramePoint3D(referenceFrame, 1.0, 1.0, 1.0);
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
      trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testIllegalParameter3()
   {
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      YoParabolicTrajectoryGenerator trajectoryGenerator = null;

      try
      {
         double intermediateParameter = 0.7;
         FramePoint3D initialPosition = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
         FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame, 0.5, 0.5, 2.5);
         FramePoint3D finalPosition = new FramePoint3D(referenceFrame, 1.0, 1.0, 1.0);
         YoVariableRegistry registry = new YoVariableRegistry("registry");
         trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
         trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);
      }
      catch (RuntimeException e)
      {
         fail();
      }

      FramePoint3D positionToPack = new FramePoint3D(referenceFrame);
      trajectoryGenerator.getPosition(positionToPack, 1.1);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApex()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInitialVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
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
      EuclidCoreTestTools.assertTuple3DEquals(initialPosition.getPoint(), initialPositionBack.getPoint(), delta);
      EuclidCoreTestTools.assertTuple3DEquals(initialVelocity.getVector(), initialVelocityBack.getVector(), delta);
      EuclidCoreTestTools.assertTuple3DEquals(finalPosition.getPoint(), finalPositionBack.getPoint(), delta);
   }
}
