package us.ihmc.robotics.math.trajectories;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePointTest;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVectorTest;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantPositionProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;


public class CirclePositionTrajectoryGeneratorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testVersusNumericalDifferentiation()
   {
      Random random = new Random(12522535L);

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint offset = new FramePoint(referenceFrame, 1.0, 2.0, 3.0);
      double rotationAngle = 2.0 * Math.PI;
      double trajectoryTime = 1.0;

      CirclePositionTrajectoryGenerator circleTrajectory = createTrajectory(referenceFrame, offset, rotationAngle, trajectoryTime);
      circleTrajectory.initialize();

      double deltaT = 1e-8;
      double tInitial = random.nextDouble() - deltaT;
      double tFinal = tInitial + deltaT;

      FramePoint position = new FramePoint();
      FrameVector velocity = new FrameVector();
      FrameVector acceleration = new FrameVector();

      circleTrajectory.compute(tInitial);
      circleTrajectory.getPosition(position);
      circleTrajectory.getVelocity(velocity);
      circleTrajectory.getAcceleration(acceleration);

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();

      circleTrajectory.compute(tFinal);
      circleTrajectory.getPosition(position2);
      circleTrajectory.getVelocity(velocity2);

      FrameVector numericallyDifferentiatedVelocity = numericallyDifferentiate(deltaT, position, position2);
      FrameVectorTest.assertFrameVectorEquals(velocity, numericallyDifferentiatedVelocity, 1e-5);

      FrameVector numericallyDifferentiatedAcceleration = numericallyDifferentiate(deltaT, velocity, velocity2);
      FrameVectorTest.assertFrameVectorEquals(acceleration, numericallyDifferentiatedAcceleration, 1e-4);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInitialAndFinalPositionFullCircle()
   {
      Random random = new Random(125125L);


      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint offset = new FramePoint(referenceFrame, RandomGeometry.nextVector3D(random));
      double rotationAngle = 2.0 * Math.PI;
      double trajectoryTime = random.nextDouble();

      CirclePositionTrajectoryGenerator circleTrajectory = createTrajectory(referenceFrame, offset, rotationAngle, trajectoryTime);
      circleTrajectory.initialize();

      FramePoint position = new FramePoint();
      FrameVector velocity = new FrameVector();
      FrameVector acceleration = new FrameVector();
      FrameVector zero = new FrameVector(referenceFrame);

      circleTrajectory.compute(0.0);
      circleTrajectory.getPosition(position);
      circleTrajectory.getVelocity(velocity);

      FramePointTest.assertFramePointEquals(offset, position, 1e-12);
      FrameVectorTest.assertFrameVectorEquals(zero, velocity, 1e-12);

      circleTrajectory.compute(trajectoryTime);
      circleTrajectory.getPosition(position);
      circleTrajectory.getVelocity(velocity);
      circleTrajectory.getAcceleration(acceleration);

      FramePointTest.assertFramePointEquals(offset, position, 1e-12);
      FrameVectorTest.assertFrameVectorEquals(zero, velocity, 1e-12);
      FrameVectorTest.assertFrameVectorEquals(zero, acceleration, 1e-12);
   }

   private FrameVector numericallyDifferentiate(double deltaT, FrameVector velocity, FrameVector velocity2)
   {
      FrameVector ret = new FrameVector(velocity2);
      ret.sub(velocity);
      ret.scale(1.0 / deltaT);

      return ret;
   }

   private FrameVector numericallyDifferentiate(double deltaT, FramePoint position, FramePoint position2)
   {
      FrameVector ret = new FrameVector(position2);
      ret.sub(position);
      ret.scale(1.0 / deltaT);

      return ret;
   }

   private CirclePositionTrajectoryGenerator createTrajectory(ReferenceFrame referenceFrame, FramePoint offset, double rotationAngle, double trajectoryTime)
   {
      PositionProvider initialPositionProvider = new ConstantPositionProvider(offset);
      DoubleProvider desiredRotationAngleProvider = new ConstantDoubleProvider(rotationAngle);
      YoVariableRegistry parentRegistry = new YoVariableRegistry("CirclePositionTrajectoryGeneratorTest");

      return new CirclePositionTrajectoryGenerator("", referenceFrame, new ConstantDoubleProvider(trajectoryTime), initialPositionProvider, parentRegistry,
              desiredRotationAngleProvider);
   }
}
