package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class YoParabolicTrajectoryGeneratorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConditions()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint intermediatePosition = new FramePoint(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint finalPosition = new FramePoint(referenceFrame, 1.0, 1.0, 1.0);
      double intermediateParameter = 0.5;

      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
      trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);

      double delta = 1e-10;
      FramePoint positionToPack = new FramePoint(referenceFrame);

      trajectoryGenerator.getPosition(positionToPack, 0.0);
      JUnitTools.assertTuple3dEquals(initialPosition.getVectorCopy(), positionToPack.getVectorCopy(), delta);

      trajectoryGenerator.getPosition(positionToPack, intermediateParameter);
      JUnitTools.assertTuple3dEquals(intermediatePosition.getVectorCopy(), positionToPack.getVectorCopy(), delta);

      trajectoryGenerator.getPosition(positionToPack, 1.0);
      JUnitTools.assertTuple3dEquals(finalPosition.getVectorCopy(), positionToPack.getVectorCopy(), delta);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testIllegalParameter1()
   {
      double intermediateParameter = 1.1;
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint intermediatePosition = new FramePoint(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint finalPosition = new FramePoint(referenceFrame, 1.0, 1.0, 1.0);
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
      FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint intermediatePosition = new FramePoint(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint finalPosition = new FramePoint(referenceFrame, 1.0, 1.0, 1.0);
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
         FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
         FramePoint intermediatePosition = new FramePoint(referenceFrame, 0.5, 0.5, 2.5);
         FramePoint finalPosition = new FramePoint(referenceFrame, 1.0, 1.0, 1.0);
         YoVariableRegistry registry = new YoVariableRegistry("registry");
         trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
         trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);
      }
      catch (RuntimeException e)
      {
         fail();
      }

      FramePoint positionToPack = new FramePoint(referenceFrame);
      trajectoryGenerator.getPosition(positionToPack, 1.1);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApex()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      FramePoint intermediatePosition = new FramePoint(referenceFrame, 0.5, 0.5, 2.5);
      FramePoint finalPosition = new FramePoint(referenceFrame, 1.0, 1.0, 0.0);
      double intermediateParameter = 0.5;

      YoParabolicTrajectoryGenerator trajectoryGenerator = new YoParabolicTrajectoryGenerator("test", referenceFrame, registry);
      trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);


      double delta = 1e-10;
      FramePoint positionToPack = new FramePoint(referenceFrame);
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
         FramePoint initialPosition = new FramePoint(referenceFrame, RandomTools.generateRandomVector(random));
         FramePoint intermediatePosition = new FramePoint(referenceFrame, RandomTools.generateRandomVector(random));
         FramePoint finalPosition = new FramePoint(referenceFrame, RandomTools.generateRandomVector(random));
         double intermediateParameter = random.nextDouble();
         trajectoryGenerator.initialize(initialPosition, intermediatePosition, finalPosition, intermediateParameter);

         FramePoint position1 = new FramePoint(referenceFrame);
         FramePoint position2 = new FramePoint(referenceFrame);

         double dt = 1e-9;
         double parameter = random.nextDouble();

         trajectoryGenerator.getPosition(position1, parameter);
         trajectoryGenerator.getPosition(position2, parameter + dt);

         FrameVector numericalVelocity = new FrameVector(position2);
         numericalVelocity.sub(position1);
         numericalVelocity.scale(1.0 / dt);

         FrameVector velocityFromTrajectoryGenerator = new FrameVector(referenceFrame);
         trajectoryGenerator.getVelocity(velocityFromTrajectoryGenerator, parameter);

         double delta = 1e-4;
         JUnitTools.assertTuple3dEquals(numericalVelocity.getVectorCopy(), velocityFromTrajectoryGenerator.getVectorCopy(), delta);
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

      FramePoint initialPosition = new FramePoint(referenceFrame, RandomTools.generateRandomVector(random));
      FrameVector initialVelocity = new FrameVector(referenceFrame, RandomTools.generateRandomVector(random));
      FramePoint finalPosition = new FramePoint(referenceFrame, RandomTools.generateRandomVector(random));
      trajectoryGenerator.initialize(initialPosition, initialVelocity, finalPosition);

      FramePoint initialPositionBack = new FramePoint(referenceFrame);
      trajectoryGenerator.getPosition(initialPositionBack, 0.0);

      FrameVector initialVelocityBack = new FrameVector(referenceFrame);
      trajectoryGenerator.getVelocity(initialVelocityBack, 0.0);

      FramePoint finalPositionBack = new FramePoint(referenceFrame);
      trajectoryGenerator.getPosition(finalPositionBack, 1.0);

      double delta = 0.0;
      JUnitTools.assertTuple3dEquals(initialPosition.getPoint(), initialPositionBack.getPoint(), delta);
      JUnitTools.assertTuple3dEquals(initialVelocity.getVector(), initialVelocityBack.getVector(), delta);
      JUnitTools.assertTuple3dEquals(finalPosition.getPoint(), finalPositionBack.getPoint(), delta);
   }
}
